let videoRef = null;
let cvReady = false;
let prevGray = null;
let prevPts = null;
let lastHomography = null;

const MAX_CORNERS = 100;
const QUALITY_LEVEL = 0.01;
const MIN_DISTANCE = 10;
const LOST_THRESHOLD = 20;
const OK_THRESHOLD = 50;

function loadOpenCv() {
  return new Promise((resolve, reject) => {
    if (window.cv && typeof window.cv.Mat === 'function') {
      cvReady = true;
      resolve();
      return;
    }

    const script = document.createElement('script');
    script.src = 'https://docs.opencv.org/4.x/opencv.js';
    script.async = true;
    script.onerror = () => reject(new Error('Failed to load OpenCV.js'));
    script.onload = () => {
      if (!window.cv) {
        reject(new Error('OpenCV global not found after script load'));
        return;
      }
      cv.onRuntimeInitialized = () => {
        cvReady = true;
        resolve();
      };
    };
    document.head.appendChild(script);
  });
}

// Detect corner-like features that are stable anchors for SLAM tracking.
function detectFeatures(gray) {
  const corners = new cv.Mat();
  const mask = new cv.Mat();

  cv.goodFeaturesToTrack(
    gray,
    corners,
    MAX_CORNERS,
    QUALITY_LEVEL,
    MIN_DISTANCE,
    mask
  );

  mask.delete();
  return corners;
}

function matPointCount(pointsMat) {
  if (!pointsMat || typeof pointsMat.rows !== 'number' || typeof pointsMat.cols !== 'number') {
    return 0;
  }
  // OpenCV.js may return point vectors as Nx1 or 1xN; both represent N points.
  return pointsMat.rows * pointsMat.cols;
}

function replacePrevGray(currentGray) {
  if (prevGray) {
    prevGray.delete();
  }
  // Keep an owned copy for the next optical flow step.
  prevGray = currentGray.clone();
}

function getHomographyStatus(confidence) {
  if (confidence > 0.6) {
    return 'stable';
  }
  if (confidence >= 0.3) {
    return 'medium';
  }
  return 'unstable';
}

function estimateHomography(prevPoints, trackedPoints) {
  const emptyResult = {
    H: null,
    inliers: 0,
    confidence: 0,
    status: 'unstable'
  };

  // Need enough matched pairs and strict source/destination alignment.
  if (prevPoints.length !== trackedPoints.length) {
    return emptyResult;
  }

  const totalMatches = trackedPoints.length / 2;
  if (totalMatches < 20) {
    return emptyResult;
  }

  const srcPts = cv.matFromArray(totalMatches, 1, cv.CV_32FC2, prevPoints);
  const dstPts = cv.matFromArray(totalMatches, 1, cv.CV_32FC2, trackedPoints);
  const inlierMask = new cv.Mat();

  const H = cv.findHomography(srcPts, dstPts, cv.RANSAC, 3.0, inlierMask);

  srcPts.delete();
  dstPts.delete();

  if (!H || H.rows === 0 || H.cols === 0) {
    if (H) {
      H.delete();
    }
    inlierMask.delete();
    return emptyResult;
  }

  let inliers = 0;
  const maskCount = inlierMask.rows * inlierMask.cols;
  for (let i = 0; i < maskCount; i++) {
    if (inlierMask.data[i]) {
      inliers += 1;
    }
  }
  inlierMask.delete();

  const confidence = totalMatches > 0 ? inliers / totalMatches : 0;
  const status = getHomographyStatus(confidence);

  return {
    H,
    inliers,
    confidence,
    status
  };
}

function matToFlatArray(mat) {
  const data = mat.data64F || mat.data32F;
  if (data && data.length > 0) {
    return Array.from(data);
  }

  // Fallback for OpenCV.js builds where typed array views are unavailable.
  const values = [];
  for (let r = 0; r < mat.rows; r++) {
    for (let c = 0; c < mat.cols; c++) {
      if (typeof mat.doubleAt === 'function') {
        values.push(mat.doubleAt(r, c));
      } else if (typeof mat.floatAt === 'function') {
        values.push(mat.floatAt(r, c));
      } else {
        return [];
      }
    }
  }
  return values;
}

function estimatePoseFromHomography(H, canvasWidth, canvasHeight) {
  const emptyPose = {
    R: [],
    t: [],
    valid: false
  };

  if (!H || canvasWidth <= 0 || canvasHeight <= 0) {
    return emptyPose;
  }

  const fx = canvasWidth;
  const fy = canvasWidth;
  const cx = canvasWidth / 2;
  const cy = canvasHeight / 2;

  const K = cv.matFromArray(3, 3, cv.CV_64F, [
    fx, 0, cx,
    0, fy, cy,
    0, 0, 1
  ]);

  const rotations = new cv.MatVector();
  const translations = new cv.MatVector();
  const normals = new cv.MatVector();
  const h64 = new cv.Mat();

  let solutionCount = 0;
  try {
    // Some mobile OpenCV.js builds are strict about homography depth.
    H.convertTo(h64, cv.CV_64F);
    solutionCount = cv.decomposeHomographyMat(h64, K, rotations, translations, normals);
  } catch (error) {
    h64.delete();
    K.delete();
    rotations.delete();
    translations.delete();
    normals.delete();
    return emptyPose;
  }

  if (solutionCount <= 0) {
    h64.delete();
    K.delete();
    rotations.delete();
    translations.delete();
    normals.delete();
    return emptyPose;
  }

  const rotationMat = rotations.get(0);
  const translationMat = translations.get(0);

  const rotationFlat = matToFlatArray(rotationMat);
  const translationFlat = matToFlatArray(translationMat);

  const R = rotationFlat.length >= 9
    ? [
      [rotationFlat[0], rotationFlat[1], rotationFlat[2]],
      [rotationFlat[3], rotationFlat[4], rotationFlat[5]],
      [rotationFlat[6], rotationFlat[7], rotationFlat[8]]
    ]
    : [];
  const t = translationFlat.length >= 3
    ? [translationFlat[0], translationFlat[1], translationFlat[2]]
    : [];

  rotationMat.delete();
  translationMat.delete();
  h64.delete();
  K.delete();
  rotations.delete();
  translations.delete();
  normals.delete();

  return {
    R,
    t,
    valid: R.length === 3 && t.length === 3
  };
}

function estimateEssentialPose(prevPoints, trackedPoints, width, height) {
  const emptyPose = {
    R: [],
    t: [],
    inliers: 0,
    confidence: 0,
    valid: false,
    status: 'unstable'
  };

  if (prevPoints.length !== trackedPoints.length) {
    return emptyPose;
  }
  if (width <= 0 || height <= 0) {
    return emptyPose;
  }

  const N = trackedPoints.length / 2;
  if (N < 20) {
    return emptyPose;
  }

  const srcPts = cv.matFromArray(N, 1, cv.CV_32FC2, prevPoints);
  const dstPts = cv.matFromArray(N, 1, cv.CV_32FC2, trackedPoints);

  const fx = width;
  const fy = width;
  const cx = width / 2;
  const cy = height / 2;

  const K = cv.matFromArray(3, 3, cv.CV_64F, [
    fx, 0, cx,
    0, fy, cy,
    0, 0, 1
  ]);

  const mask = new cv.Mat();
  const E = cv.findEssentialMat(
    srcPts,
    dstPts,
    K,
    cv.RANSAC,
    0.999,
    1.0,
    mask
  );

  if (!E || E.rows === 0 || E.cols === 0) {
    if (E) {
      E.delete();
    }
    srcPts.delete();
    dstPts.delete();
    K.delete();
    mask.delete();
    return emptyPose;
  }

  const R = new cv.Mat();
  const t = new cv.Mat();

  let inliers = 0;
  try {
    inliers = cv.recoverPose(
      E,
      srcPts,
      dstPts,
      K,
      R,
      t,
      mask
    );
  } catch (error) {
    srcPts.delete();
    dstPts.delete();
    K.delete();
    mask.delete();
    E.delete();
    R.delete();
    t.delete();
    return emptyPose;
  }

  const rotationFlat = matToFlatArray(R);
  const translationFlat = matToFlatArray(t);

  const rotation = rotationFlat.length >= 9
    ? [
      [rotationFlat[0], rotationFlat[1], rotationFlat[2]],
      [rotationFlat[3], rotationFlat[4], rotationFlat[5]],
      [rotationFlat[6], rotationFlat[7], rotationFlat[8]]
    ]
    : [];
  const translation = translationFlat.length >= 3
    ? [translationFlat[0], translationFlat[1], translationFlat[2]]
    : [];

  const confidence = N > 0 ? inliers / N : 0;
  const translationMagnitude = translation.length === 3
    ? Math.hypot(translation[0], translation[1], translation[2])
    : 0;
  const valid = (
    inliers > 30 &&
    confidence > 0.5 &&
    translationMagnitude > 0.001 &&
    rotation.length === 3 &&
    translation.length === 3
  );

  srcPts.delete();
  dstPts.delete();
  K.delete();
  mask.delete();
  E.delete();
  R.delete();
  t.delete();

  return {
    R: rotation,
    t: translation,
    inliers,
    confidence,
    valid,
    status: confidence > 0.6 ? 'stable' : 'unstable'
  };
}

async function initTracking(video) {
  videoRef = video;
  try {
    await loadOpenCv();
    console.log('OpenCV.js ready');
  } catch (error) {
    console.error(error);
    throw error;
  }
}

function processTrackingFrame(canvas, options = {}) {
  const shouldForceRedetect = Boolean(options.forceRedetect);

  if (!videoRef || !cvReady || !window.cv) {
    return {
      loading: true,
      trackedCount: 0,
      trackedPoints: [],
      prevPoints: [],
      status: 'Tracking LOST',
      homography: {
        H: null,
        inliers: 0,
        confidence: 0,
        status: 'unstable'
      },
      pose: {
        R: [],
        t: [],
        valid: false
      },
      poseEssential: {
        R: [],
        t: [],
        inliers: 0,
        confidence: 0,
        valid: false,
        status: 'unstable'
      }
    };
  }

  // Convert rendered canvas frame to Mat so OpenCV can operate on it.
  const rgba = cv.imread(canvas);
  const gray = new cv.Mat();
  cv.cvtColor(rgba, gray, cv.COLOR_RGBA2GRAY);

  let trackedData = {
    loading: false,
    trackedCount: 0,
    trackedPoints: [],
    prevPoints: [],
    status: 'Tracking LOST',
    homography: {
      H: null,
      inliers: 0,
      confidence: 0,
      status: 'unstable'
    },
    pose: {
      R: [],
      t: [],
      valid: false
    },
    poseEssential: {
      R: [],
      t: [],
      inliers: 0,
      confidence: 0,
      valid: false,
      status: 'unstable'
    }
  };

  try {
    let pointsToUse = prevPts;
    let redetectedThisFrame = false;

    if (shouldForceRedetect) {
      if (prevPts) {
        prevPts.delete();
        prevPts = null;
      }
      if (prevGray) {
        prevGray.delete();
        prevGray = null;
      }
      console.log('Manual feature re-detection requested');
    }

    // First frame or recovery path: detect strong corners to bootstrap tracking.
    if (!prevGray || !prevPts || matPointCount(prevPts) < LOST_THRESHOLD) {
      if (prevPts) {
        prevPts.delete();
      }
      pointsToUse = detectFeatures(gray);
      prevPts = pointsToUse;
      redetectedThisFrame = true;
      console.log(`Re-detected features: ${matPointCount(prevPts)}`);
    }

    let trackedCount = 0;
    let trackedPoints = [];
    let prevPoints = [];

    if (!redetectedThisFrame && prevGray && pointsToUse && matPointCount(pointsToUse) > 0) {
      const nextPts = new cv.Mat();
      const status = new cv.Mat();
      const err = new cv.Mat();
      const winSize = new cv.Size(21, 21);
      const criteria = new cv.TermCriteria(
        cv.TermCriteria_EPS + cv.TermCriteria_COUNT,
        30,
        0.01
      );

      // Lucas-Kanade optical flow links feature positions frame-to-frame.
      cv.calcOpticalFlowPyrLK(
        prevGray,
        gray,
        pointsToUse,
        nextPts,
        status,
        err,
        winSize,
        3,
        criteria
      );

      const statusCount = status.rows * status.cols;
      for (let i = 0; i < statusCount; i++) {
        if (status.data[i] === 1) {
          const prevX = pointsToUse.data32F[i * 2];
          const prevY = pointsToUse.data32F[i * 2 + 1];
          const currX = nextPts.data32F[i * 2];
          const currY = nextPts.data32F[i * 2 + 1];

          prevPoints.push(prevX, prevY);
          trackedPoints.push(currX, currY);
          trackedCount += 1;
        }
      }

      if (prevPts) {
        prevPts.delete();
        prevPts = null;
      }

      if (trackedCount > 0) {
        prevPts = cv.matFromArray(trackedCount, 1, cv.CV_32FC2, trackedPoints);
      }

      nextPts.delete();
      status.delete();
      err.delete();
    } else if (pointsToUse && matPointCount(pointsToUse) > 0) {
      trackedCount = matPointCount(pointsToUse);
      trackedPoints = Array.from(pointsToUse.data32F);
    }

    // If too few tracks survive, refresh features so tracking can recover.
    if (trackedCount < LOST_THRESHOLD) {
      if (prevPts) {
        prevPts.delete();
      }
      prevPts = detectFeatures(gray);
      trackedCount = matPointCount(prevPts);
      trackedPoints = Array.from(prevPts.data32F);
      prevPoints = [];
      console.log(`Re-detected features: ${matPointCount(prevPts)}`);
    }

    replacePrevGray(gray);

    let statusText = 'Tracking recovering...';
    if (trackedCount < LOST_THRESHOLD) {
      statusText = 'Tracking LOST';
    } else if (trackedCount > OK_THRESHOLD) {
      statusText = 'Tracking OK';
    }

    const homographyEstimate = estimateHomography(prevPoints, trackedPoints);

    // Keep one owned homography Mat alive to avoid leaking old frame results.
    if (lastHomography) {
      lastHomography.delete();
      lastHomography = null;
    }
    if (homographyEstimate.H) {
      lastHomography = homographyEstimate.H;
      console.log(
        `Homography updated | inliers: ${homographyEstimate.inliers}, confidence: ${homographyEstimate.confidence.toFixed(2)}, status: ${homographyEstimate.status}`
      );
    }

    const poseEstimate = estimatePoseFromHomography(
      lastHomography,
      canvas.width,
      canvas.height
    );
    const poseValidByStability = (
      homographyEstimate.status === 'stable' && homographyEstimate.confidence > 0.6
    );
    const pose = {
      R: poseEstimate.R,
      t: poseEstimate.t,
      valid: poseEstimate.valid && poseValidByStability
    };
    if (homographyEstimate.status === 'stable' && !pose.valid) {
      console.log('Pose decomposition unavailable for current stable homography');
    }

    const essentialPose = estimateEssentialPose(
      prevPoints,
      trackedPoints,
      canvas.width,
      canvas.height
    );
    if (essentialPose.valid) {
      console.log('Essential Pose:', {
        inliers: essentialPose.inliers,
        confidence: essentialPose.confidence.toFixed(2),
        t: essentialPose.t,
        valid: essentialPose.valid
      });
    }

    trackedData = {
      loading: false,
      trackedCount,
      trackedPoints,
      prevPoints,
      status: statusText,
      homography: {
        H: lastHomography,
        inliers: homographyEstimate.inliers,
        confidence: homographyEstimate.confidence,
        status: homographyEstimate.status
      },
      pose,
      poseEssential: essentialPose
    };
  } catch (cvError) {
    console.error('Frame tracking error:', cvError);
  }

  rgba.delete();
  gray.delete();

  return trackedData;
}

export { initTracking, processTrackingFrame };
