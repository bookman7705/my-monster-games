let videoRef = null;
let cvReady = false;
let prevGray = null;
let prevFramePts = [];
let lastHomography = null;
let lostFrames = 0;
let trackingState = 'Tracking LOST';
let smoothedHomographyPose = null;
let smoothedEssentialPose = null;
let lastValidPose = null;
let lastPoseSource = null;

// FIX: More robust feature detection defaults for mobile scenes.
const MAX_CORNERS = 180;
const QUALITY_LEVEL = 0.005;
const MIN_DISTANCE = 7;
const LOST_THRESHOLD = 20;
const OK_THRESHOLD = 50;
const STABLE_THRESHOLD = 80;
// FIX: Increase hysteresis to avoid frequent reset loops.
const LOST_FRAME_THRESHOLD = 10;
// FIX: LK error gate was too strict and dropped valid tracks.
const MAX_LK_ERROR = 80;
// FIX: Use a stricter reset threshold than HUD LOST threshold.
const RESET_TRACK_THRESHOLD = 10;
const MIN_POSE_POINTS = 20;
const MIN_POINT_SPREAD = 30;
const POSE_SMOOTH_ALPHA = 0.7;
const MAX_POSE_TRANSLATION_DELTA = 2.5;
const MAX_POSE_ROTATION_DELTA = 1.5;
const MAX_REPROJECTION_ERROR = 5;
// FIX: Reject abrupt quality drops instead of instantly replacing stable tracks.
const MIN_TRACK_RETENTION_RATIO = 0.2;

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

function pointsToFlatArray(points) {
  const flat = [];
  for (let i = 0; i < points.length; i++) {
    flat.push(points[i].x, points[i].y);
  }
  return flat;
}

function flatArrayToPoints(flat) {
  const points = [];
  for (let i = 0; i + 1 < flat.length; i += 2) {
    points.push({ x: flat[i], y: flat[i + 1] });
  }
  return points;
}

function pointsToMatNx2(points, depth = cv.CV_64F) {
  return cv.matFromArray(points.length, 2, depth, pointsToFlatArray(points));
}

function pointsToMatCv32FC2(points) {
  return cv.matFromArray(points.length, 1, cv.CV_32FC2, pointsToFlatArray(points));
}

function matToPoints(pointsMat) {
  return flatArrayToPoints(matToFlatArray(pointsMat));
}

function getMatPointAt(pointsMat, index) {
  const data32 = pointsMat.data32F;
  if (data32 && data32.length >= (index * 2 + 2)) {
    return { x: data32[index * 2], y: data32[index * 2 + 1] };
  }

  const data64 = pointsMat.data64F;
  if (data64 && data64.length >= (index * 2 + 2)) {
    return { x: data64[index * 2], y: data64[index * 2 + 1] };
  }

  if (typeof pointsMat.floatAt === 'function') {
    return { x: pointsMat.floatAt(index, 0), y: pointsMat.floatAt(index, 1) };
  }

  if (typeof pointsMat.doubleAt === 'function') {
    return { x: pointsMat.doubleAt(index, 0), y: pointsMat.doubleAt(index, 1) };
  }

  return { x: NaN, y: NaN };
}

function getMatScalarAt(mat, index) {
  if (mat.data32F && mat.data32F.length > index) {
    return mat.data32F[index];
  }
  if (mat.data64F && mat.data64F.length > index) {
    return mat.data64F[index];
  }
  if (mat.data && mat.data.length > index) {
    return mat.data[index];
  }
  if (typeof mat.floatAt === 'function') {
    return mat.floatAt(index, 0);
  }
  if (typeof mat.doubleAt === 'function') {
    return mat.doubleAt(index, 0);
  }
  return NaN;
}

function pointSpread(points) {
  if (!points || points.length === 0) {
    return 0;
  }
  let minX = points[0].x;
  let maxX = points[0].x;
  let minY = points[0].y;
  let maxY = points[0].y;
  for (let i = 1; i < points.length; i++) {
    const p = points[i];
    if (p.x < minX) minX = p.x;
    if (p.x > maxX) maxX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.y > maxY) maxY = p.y;
  }
  return Math.hypot(maxX - minX, maxY - minY);
}

function hasSufficientPoseInput(prevPoints, currPoints) {
  return (
    prevPoints.length >= MIN_POSE_POINTS &&
    currPoints.length >= MIN_POSE_POINTS &&
    pointSpread(prevPoints) >= MIN_POINT_SPREAD &&
    pointSpread(currPoints) >= MIN_POINT_SPREAD
  );
}

// FIX: Avoid hard failure on unequal array sizes by pairing only valid overlap.
function alignPointPairs(prevPoints, currPoints) {
  const pairCount = Math.min(prevPoints.length, currPoints.length);
  return {
    prev: prevPoints.slice(0, pairCount),
    curr: currPoints.slice(0, pairCount),
    pairCount
  };
}

function smoothArray(nextValues, prevValues, alpha) {
  if (!prevValues || prevValues.length !== nextValues.length) {
    return nextValues.slice();
  }
  const smoothed = [];
  for (let i = 0; i < nextValues.length; i++) {
    smoothed.push((alpha * nextValues[i]) + ((1 - alpha) * prevValues[i]));
  }
  return smoothed;
}

function smoothPose(nextPose, prevPose) {
  if (!nextPose.valid) {
    return nextPose;
  }

  if (!prevPose || !prevPose.valid) {
    return {
      R: nextPose.R.map((row) => row.slice()),
      t: nextPose.t.slice(),
      valid: true
    };
  }

  const nextRFlat = [
    nextPose.R[0][0], nextPose.R[0][1], nextPose.R[0][2],
    nextPose.R[1][0], nextPose.R[1][1], nextPose.R[1][2],
    nextPose.R[2][0], nextPose.R[2][1], nextPose.R[2][2]
  ];
  const prevRFlat = [
    prevPose.R[0][0], prevPose.R[0][1], prevPose.R[0][2],
    prevPose.R[1][0], prevPose.R[1][1], prevPose.R[1][2],
    prevPose.R[2][0], prevPose.R[2][1], prevPose.R[2][2]
  ];
  const smoothedRFlat = smoothArray(nextRFlat, prevRFlat, POSE_SMOOTH_ALPHA);
  const smoothedR = [
    [smoothedRFlat[0], smoothedRFlat[1], smoothedRFlat[2]],
    [smoothedRFlat[3], smoothedRFlat[4], smoothedRFlat[5]],
    [smoothedRFlat[6], smoothedRFlat[7], smoothedRFlat[8]]
  ];
  const smoothedT = smoothArray(nextPose.t, prevPose.t, POSE_SMOOTH_ALPHA);

  return {
    R: smoothedR,
    t: smoothedT,
    valid: true
  };
}

function classifyTrackingStatus(trackedCount) {
  if (trackingState === 'Tracking LOST') {
    if (trackedCount >= LOST_THRESHOLD) {
      trackingState = trackedCount > STABLE_THRESHOLD ? 'Tracking STABLE' : 'Tracking OK';
    }
  } else if (trackingState === 'Tracking STABLE') {
    if (trackedCount < OK_THRESHOLD) {
      trackingState = trackedCount < LOST_THRESHOLD ? 'Tracking LOST' : 'Tracking OK';
    }
  } else {
    if (trackedCount < LOST_THRESHOLD) {
      trackingState = 'Tracking LOST';
    } else if (trackedCount > STABLE_THRESHOLD) {
      trackingState = 'Tracking STABLE';
    }
  }
  return trackingState;
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

  let points = matToPoints(corners);
  // FIX: Retry with softer settings when too few features are detected.
  if (points.length < 40) {
    corners.delete();
    const cornersRetry = new cv.Mat();
    cv.goodFeaturesToTrack(
      gray,
      cornersRetry,
      Math.max(MAX_CORNERS, 220),
      0.003,
      5,
      mask
    );
    points = matToPoints(cornersRetry);
    cornersRetry.delete();
  }
  corners.delete();
  mask.delete();
  return points;
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

  // FIX: Build robust pair alignment instead of requiring exact array equality.
  const aligned = alignPointPairs(prevPoints, trackedPoints);
  const totalMatches = aligned.pairCount;
  if (totalMatches < 20) {
    return emptyResult;
  }

  const srcPts = pointsToMatCv32FC2(aligned.prev);
  const dstPts = pointsToMatCv32FC2(aligned.curr);
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

  const focal = 0.9 * Math.max(canvasWidth, canvasHeight);
  const fx = focal;
  const fy = focal;
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
    console.log('[POSE DEBUG] decomposition failed or returned empty result');
    h64.delete();
    K.delete();
    rotations.delete();
    translations.delete();
    normals.delete();
    return emptyPose;
  }

  console.log('[POSE DEBUG] solutions:', solutionCount);

  if (solutionCount <= 0) {
    console.log('[POSE DEBUG] decomposition failed or returned empty result');
    h64.delete();
    K.delete();
    rotations.delete();
    translations.delete();
    normals.delete();
    return emptyPose;
  }

  let R = [];
  let t = [];
  let rotationFlat = [];
  let translationFlat = [];
  const solutionLimit = Math.min(solutionCount, 2);
  for (let i = 0; i < solutionLimit; i++) {
    const rotationMat = rotations.get(i);
    const translationMat = translations.get(i);

    rotationFlat = matToFlatArray(rotationMat);
    translationFlat = matToFlatArray(translationMat);
    console.log('[POSE DEBUG] R:', rotationFlat);
    console.log('[POSE DEBUG] t:', translationFlat);

    if (rotationFlat.length === 9 && translationFlat.length === 3) {
      R = [
        [rotationFlat[0], rotationFlat[1], rotationFlat[2]],
        [rotationFlat[3], rotationFlat[4], rotationFlat[5]],
        [rotationFlat[6], rotationFlat[7], rotationFlat[8]]
      ];
      t = [translationFlat[0], translationFlat[1], translationFlat[2]];
      rotationMat.delete();
      translationMat.delete();
      break;
    }

    rotationMat.delete();
    translationMat.delete();
  }

  if (R.length !== 3 || t.length !== 3) {
    console.log('[POSE DEBUG] decomposition failed or returned empty result');
  }

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

  if (width <= 0 || height <= 0) {
    return emptyPose;
  }

  // FIX: Align pairs by overlap to prevent index mismatch failures.
  const aligned = alignPointPairs(prevPoints, trackedPoints);
  const N = aligned.pairCount;
  if (N < 20) {
    return emptyPose;
  }

  let srcPts = null;
  let dstPts = null;
  let srcPtsFallback = null;
  let dstPtsFallback = null;
  let K = null;
  let mask = null;
  let E = null;
  let R = null;
  let t = null;

  const fx = width * 0.9;
  const fy = height * 0.9;
  const cx = width / 2;
  const cy = height / 2;

  try {
    // Primary format: Nx2 tends to be more reliable across mobile OpenCV.js builds.
    srcPts = pointsToMatNx2(aligned.prev, cv.CV_64F);
    dstPts = pointsToMatNx2(aligned.curr, cv.CV_64F);

    K = cv.matFromArray(3, 3, cv.CV_64F, [
      fx, 0, cx,
      0, fy, cy,
      0, 0, 1
    ]);

    mask = new cv.Mat();
    try {
      E = cv.findEssentialMat(
        srcPts,
        dstPts,
        K,
        cv.RANSAC,
        0.999,
        1.0,
        mask
      );
    } catch (primaryError) {
      // Fallback format: Nx1 CV_32FC2 for builds that prefer 2-channel point mats.
      srcPtsFallback = pointsToMatCv32FC2(aligned.prev);
      dstPtsFallback = pointsToMatCv32FC2(aligned.curr);
      E = cv.findEssentialMat(
        srcPtsFallback,
        dstPtsFallback,
        K,
        cv.RANSAC,
        0.999,
        1.0,
        mask
      );
      srcPts.delete();
      dstPts.delete();
      srcPts = srcPtsFallback;
      dstPts = dstPtsFallback;
      srcPtsFallback = null;
      dstPtsFallback = null;
    }

    if (!E || E.rows === 0 || E.cols === 0) {
      console.log('Essential pose rejected: empty essential matrix');
      return emptyPose;
    }

    R = new cv.Mat();
    t = new cv.Mat();

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
      console.log('Essential pose rejected: recoverPose failed', error);
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
    const valid = (
      inliers > 15 &&
      confidence > 0.3 &&
      rotation.length === 3 &&
      translation.length === 3
    );

    if (!valid) {
      console.log('Essential pose rejected:', {
        inliers,
        confidence,
        rotationValid: rotation.length === 3,
        translationValid: translation.length === 3
      });
    }

    return {
      R: rotation,
      t: translation,
      inliers,
      confidence,
      valid,
      status: confidence > 0.6 ? 'stable' : 'unstable'
    };
  } catch (error) {
    console.log('Essential pose rejected: pipeline failed', error);
    return emptyPose;
  } finally {
    if (srcPts) srcPts.delete();
    if (dstPts) dstPts.delete();
    if (srcPtsFallback) srcPtsFallback.delete();
    if (dstPtsFallback) dstPtsFallback.delete();
    if (K) K.delete();
    if (mask) mask.delete();
    if (E) E.delete();
    if (R) R.delete();
    if (t) t.delete();
  }
}

function createEmptyHomographyResult() {
  return {
    H: null,
    inliers: 0,
    confidence: 0,
    status: 'unstable'
  };
}

function createEmptyEssentialPose() {
  return {
    R: [],
    t: [],
    inliers: 0,
    confidence: 0,
    valid: false,
    status: 'unstable'
  };
}

function createIdentityPose() {
  return {
    R: [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1]
    ],
    t: [0, 0, 0],
    valid: true
  };
}

function clonePose(pose) {
  if (!pose || !pose.valid) {
    return createIdentityPose();
  }
  return {
    R: pose.R.map((row) => row.slice()),
    t: pose.t.slice(),
    valid: true
  };
}

function isPoseStructValid(pose) {
  return (
    pose &&
    pose.valid === true &&
    Array.isArray(pose.R) &&
    pose.R.length === 3 &&
    Array.isArray(pose.t) &&
    pose.t.length === 3
  );
}

function selectBestPose(homographyPose, essentialPose, currentTrackingState) {
  if (
    isPoseStructValid(homographyPose) &&
    currentTrackingState === 'Tracking STABLE' &&
    homographyPose.confidence > 0.6 &&
    homographyPose.inliers > 25
  ) {
    return { pose: homographyPose, source: 'homography', confidence: homographyPose.confidence };
  }

  if (
    isPoseStructValid(essentialPose) &&
    essentialPose.inliers > 15 &&
    essentialPose.confidence > 0.3
  ) {
    return { pose: essentialPose, source: 'essential', confidence: essentialPose.confidence };
  }

  if (lastValidPose && isPoseStructValid(lastValidPose)) {
    return { pose: lastValidPose, source: 'fallback', confidence: 0 };
  }

  return { pose: createIdentityPose(), source: 'fallback', confidence: 0 };
}

function getPoseDelta(currentPose, previousPose) {
  if (!isPoseStructValid(currentPose) || !isPoseStructValid(previousPose)) {
    return { translationDelta: 0, rotationDelta: 0 };
  }

  const translationDelta = Math.hypot(
    currentPose.t[0] - previousPose.t[0],
    currentPose.t[1] - previousPose.t[1],
    currentPose.t[2] - previousPose.t[2]
  );

  // FIX: Use relative rotation angle instead of Euclidean matrix delta.
  const Rc = currentPose.R;
  const Rp = previousPose.R;
  const RpT = [
    [Rp[0][0], Rp[1][0], Rp[2][0]],
    [Rp[0][1], Rp[1][1], Rp[2][1]],
    [Rp[0][2], Rp[1][2], Rp[2][2]]
  ];
  const Rrel = [
    [
      RpT[0][0] * Rc[0][0] + RpT[0][1] * Rc[1][0] + RpT[0][2] * Rc[2][0],
      RpT[0][0] * Rc[0][1] + RpT[0][1] * Rc[1][1] + RpT[0][2] * Rc[2][1],
      RpT[0][0] * Rc[0][2] + RpT[0][1] * Rc[1][2] + RpT[0][2] * Rc[2][2]
    ],
    [
      RpT[1][0] * Rc[0][0] + RpT[1][1] * Rc[1][0] + RpT[1][2] * Rc[2][0],
      RpT[1][0] * Rc[0][1] + RpT[1][1] * Rc[1][1] + RpT[1][2] * Rc[2][1],
      RpT[1][0] * Rc[0][2] + RpT[1][1] * Rc[1][2] + RpT[1][2] * Rc[2][2]
    ],
    [
      RpT[2][0] * Rc[0][0] + RpT[2][1] * Rc[1][0] + RpT[2][2] * Rc[2][0],
      RpT[2][0] * Rc[0][1] + RpT[2][1] * Rc[1][1] + RpT[2][2] * Rc[2][1],
      RpT[2][0] * Rc[0][2] + RpT[2][1] * Rc[1][2] + RpT[2][2] * Rc[2][2]
    ]
  ];
  const trace = Rrel[0][0] + Rrel[1][1] + Rrel[2][2];
  const cosTheta = Math.min(1, Math.max(-1, (trace - 1) / 2));
  const rotationDelta = Math.abs(Math.acos(cosTheta));

  return {
    translationDelta,
    rotationDelta
  };
}

function computeHomographyWarpError(homography, prevPoints, currPoints) {
  if (!homography || !homography.H || prevPoints.length === 0 || currPoints.length === 0) {
    return 0;
  }
  const data = homography.H.data64F || homography.H.data32F;
  if (!data || data.length < 9) {
    return Infinity;
  }

  let errorSum = 0;
  const count = Math.min(prevPoints.length, currPoints.length);
  for (let i = 0; i < count; i++) {
    const p = prevPoints[i];
    const q = currPoints[i];
    const w = (data[6] * p.x) + (data[7] * p.y) + data[8];
    if (Math.abs(w) < 1e-6) {
      continue;
    }
    const xWarp = ((data[0] * p.x) + (data[1] * p.y) + data[2]) / w;
    const yWarp = ((data[3] * p.x) + (data[4] * p.y) + data[5]) / w;
    errorSum += Math.hypot(xWarp - q.x, yWarp - q.y);
  }

  return count > 0 ? errorSum / count : 0;
}

function computeEssentialEpipolarError(pose, prevPoints, currPoints, width, height) {
  if (!isPoseStructValid(pose) || prevPoints.length === 0 || currPoints.length === 0) {
    return 0;
  }

  const focal = 0.9 * Math.max(width, height);
  const fx = focal;
  const fy = focal;
  const cx = width / 2;
  const cy = height / 2;
  const invFx = 1 / fx;
  const invFy = 1 / fy;

  const t = pose.t;
  const tx = [
    [0, -t[2], t[1]],
    [t[2], 0, -t[0]],
    [-t[1], t[0], 0]
  ];
  const R = pose.R;
  const E = [
    [
      tx[0][0] * R[0][0] + tx[0][1] * R[1][0] + tx[0][2] * R[2][0],
      tx[0][0] * R[0][1] + tx[0][1] * R[1][1] + tx[0][2] * R[2][1],
      tx[0][0] * R[0][2] + tx[0][1] * R[1][2] + tx[0][2] * R[2][2]
    ],
    [
      tx[1][0] * R[0][0] + tx[1][1] * R[1][0] + tx[1][2] * R[2][0],
      tx[1][0] * R[0][1] + tx[1][1] * R[1][1] + tx[1][2] * R[2][1],
      tx[1][0] * R[0][2] + tx[1][1] * R[1][2] + tx[1][2] * R[2][2]
    ],
    [
      tx[2][0] * R[0][0] + tx[2][1] * R[1][0] + tx[2][2] * R[2][0],
      tx[2][0] * R[0][1] + tx[2][1] * R[1][1] + tx[2][2] * R[2][1],
      tx[2][0] * R[0][2] + tx[2][1] * R[1][2] + tx[2][2] * R[2][2]
    ]
  ];

  let errorSum = 0;
  const count = Math.min(prevPoints.length, currPoints.length);
  for (let i = 0; i < count; i++) {
    const p = prevPoints[i];
    const q = currPoints[i];
    const x1 = [(p.x - cx) * invFx, (p.y - cy) * invFy, 1];
    const x2 = [(q.x - cx) * invFx, (q.y - cy) * invFy, 1];

    const line = [
      E[0][0] * x1[0] + E[0][1] * x1[1] + E[0][2] * x1[2],
      E[1][0] * x1[0] + E[1][1] * x1[1] + E[1][2] * x1[2],
      E[2][0] * x1[0] + E[2][1] * x1[1] + E[2][2] * x1[2]
    ];

    const denom = Math.hypot(line[0], line[1]);
    if (denom < 1e-9) {
      continue;
    }
    const normDistance = Math.abs((line[0] * x2[0]) + (line[1] * x2[1]) + line[2]) / denom;
    errorSum += normDistance * focal;
  }

  return count > 0 ? errorSum / count : 0;
}

function computeReprojectionError(pose, points2D, points2DNext, options = {}) {
  const source = options.source || 'fallback';
  if (source === 'homography') {
    return computeHomographyWarpError(options.homography, points2D, points2DNext);
  }
  if (source === 'essential') {
    return computeEssentialEpipolarError(
      pose,
      points2D,
      points2DNext,
      options.width || 1,
      options.height || 1
    );
  }
  return 0;
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
    const fallbackPose = lastValidPose && isPoseStructValid(lastValidPose)
      ? clonePose(lastValidPose)
      : createIdentityPose();
    return {
      loading: true,
      trackedCount: 0,
      trackedPoints: [],
      prevPoints: [],
      status: 'Tracking LOST',
      homography: createEmptyHomographyResult(),
      pose: fallbackPose,
      poseEssential: createEmptyEssentialPose(),
      poseDebug: {
        selectedSource: 'fallback',
        confidence: 0,
        reprojectionError: 0,
        wasClamped: false
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
    homography: createEmptyHomographyResult(),
    pose: createIdentityPose(),
    poseEssential: createEmptyEssentialPose(),
    poseDebug: {
      selectedSource: 'fallback',
      confidence: 0,
      reprojectionError: 0,
      wasClamped: false
    }
  };

  try {
    if (shouldForceRedetect) {
      prevFramePts = [];
      if (prevGray) {
        prevGray.delete();
        prevGray = null;
      }
      lostFrames = LOST_FRAME_THRESHOLD;
      console.log('Manual feature re-detection requested');
    }

    const previousFramePoints = prevFramePts.map((p) => ({ x: p.x, y: p.y }));
    let prevMatchedPoints = [];
    let currMatchedPoints = [];
    let nextFramePoints = [];

    // First frame or recovery path: detect strong corners to bootstrap tracking.
    if (!prevGray || previousFramePoints.length < LOST_THRESHOLD) {
      nextFramePoints = detectFeatures(gray);
      console.log(`Re-detected features: ${nextFramePoints.length}`);
    } else {
      const srcPtsMat = pointsToMatCv32FC2(previousFramePoints);
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
        srcPtsMat,
        nextPts,
        status,
        err,
        winSize,
        3,
        criteria
      );

      const statusCount = Math.min(status.rows * status.cols, previousFramePoints.length);
      for (let i = 0; i < statusCount; i++) {
        const st = getMatScalarAt(status, i);
        if (st !== 1) {
          continue;
        }

        const prevPt = previousFramePoints[i];
        const currPt = getMatPointAt(nextPts, i);
        const currX = currPt.x;
        const currY = currPt.y;
        const lkErrorValue = getMatScalarAt(err, i);
        const lkError = Number.isFinite(lkErrorValue) ? lkErrorValue : 0;

        if (
          !Number.isFinite(prevPt.x) ||
          !Number.isFinite(prevPt.y) ||
          !Number.isFinite(currX) ||
          !Number.isFinite(currY) ||
          (Number.isFinite(lkError) && lkError > MAX_LK_ERROR)
        ) {
          continue;
        }

        prevMatchedPoints.push(prevPt);
        currMatchedPoints.push({ x: currX, y: currY });
      }

      nextFramePoints = currMatchedPoints.map((p) => ({ x: p.x, y: p.y }));

      srcPtsMat.delete();
      nextPts.delete();
      status.delete();
      err.delete();
    }

    let trackedCount = nextFramePoints.length;
    // FIX: Drop low-quality abrupt updates and preserve last stable tracks.
    if (
      previousFramePoints.length > 30 &&
      trackedCount < (previousFramePoints.length * MIN_TRACK_RETENTION_RATIO)
    ) {
      nextFramePoints = previousFramePoints.map((p) => ({ x: p.x, y: p.y }));
      prevMatchedPoints = [];
      currMatchedPoints = [];
      trackedCount = nextFramePoints.length;
    }
    if (trackedCount < RESET_TRACK_THRESHOLD) {
      lostFrames += 1;
    } else {
      lostFrames = 0;
    }

    // Re-detect only after sustained loss to avoid anchor popping.
    if (lostFrames >= LOST_FRAME_THRESHOLD) {
      nextFramePoints = detectFeatures(gray);
      trackedCount = nextFramePoints.length;
      prevMatchedPoints = [];
      currMatchedPoints = [];
      lostFrames = 0;
      console.log(`Re-detected features: ${trackedCount}`);
    }

    const canEstimatePose = hasSufficientPoseInput(prevMatchedPoints, currMatchedPoints);
    const homographyEstimate = canEstimatePose
      ? estimateHomography(prevMatchedPoints, currMatchedPoints)
      : createEmptyHomographyResult();

    // FIX: Own homography result directly without unnecessary cloning.
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
    const homographyPoseRaw = {
      R: poseEstimate.R,
      t: poseEstimate.t,
      valid: (
        Array.isArray(poseEstimate.R) &&
        poseEstimate.R.length === 3 &&
        Array.isArray(poseEstimate.t) &&
        poseEstimate.t.length === 3
      ),
      inliers: homographyEstimate.inliers,
      confidence: homographyEstimate.confidence
    };
    const smoothedHomography = smoothPose(homographyPoseRaw, smoothedHomographyPose);
    if (smoothedHomography.valid) {
      smoothedHomographyPose = smoothedHomography;
    } else {
      smoothedHomographyPose = null;
    }
    if (homographyEstimate.H && !homographyPoseRaw.valid) {
      console.log('Pose decomposition unavailable for current stable homography');
    }

    const essentialRaw = canEstimatePose
      ? estimateEssentialPose(
        prevMatchedPoints,
        currMatchedPoints,
        canvas.width,
        canvas.height
      )
      : createEmptyEssentialPose();
    let essentialPose = essentialRaw;
    if (essentialRaw.valid) {
      const smoothedEssential = smoothPose(
        { R: essentialRaw.R, t: essentialRaw.t, valid: true },
        smoothedEssentialPose
      );
      smoothedEssentialPose = smoothedEssential;
      essentialPose = {
        ...essentialRaw,
        R: smoothedEssential.R,
        t: smoothedEssential.t
      };
      console.log('Essential Pose:', {
        inliers: essentialPose.inliers,
        confidence: essentialPose.confidence.toFixed(2),
        t: essentialPose.t,
        valid: essentialPose.valid
      });
    } else {
      smoothedEssentialPose = null;
    }

    const statusText = classifyTrackingStatus(trackedCount);

    const homographyPoseCandidate = {
      ...smoothedHomography,
      inliers: homographyEstimate.inliers,
      confidence: homographyEstimate.confidence
    };
    const selected = selectBestPose(
      homographyPoseCandidate,
      essentialPose,
      statusText
    );

    let selectedPose = clonePose(selected.pose);
    let selectedSource = selected.source;
    let wasClamped = false;
    let reprojectionError = computeReprojectionError(
      selectedPose,
      prevMatchedPoints,
      currMatchedPoints,
      {
        source: selectedSource,
        homography: { H: lastHomography },
        width: canvas.width,
        height: canvas.height
      }
    );

    if (Number.isFinite(reprojectionError) && reprojectionError > MAX_REPROJECTION_ERROR) {
      wasClamped = true;
      selectedSource = 'fallback';
      selectedPose = lastValidPose && isPoseStructValid(lastValidPose)
        ? clonePose(lastValidPose)
        : createIdentityPose();
      reprojectionError = 0;
    }

    if (lastValidPose && isPoseStructValid(lastValidPose)) {
      const poseDelta = getPoseDelta(selectedPose, lastValidPose);
      if (
        poseDelta.translationDelta > MAX_POSE_TRANSLATION_DELTA ||
        poseDelta.rotationDelta > MAX_POSE_ROTATION_DELTA
      ) {
        wasClamped = true;
        selectedSource = 'fallback';
        selectedPose = clonePose(lastValidPose);
      }
    }

    if (isPoseStructValid(selectedPose)) {
      lastValidPose = clonePose(selectedPose);
      lastPoseSource = selectedSource === 'fallback' ? (lastPoseSource || 'fallback') : selectedSource;
    } else {
      selectedSource = 'fallback';
      selectedPose = lastValidPose && isPoseStructValid(lastValidPose)
        ? clonePose(lastValidPose)
        : createIdentityPose();
      lastValidPose = clonePose(selectedPose);
      lastPoseSource = lastPoseSource || 'fallback';
    }

    // Swap buffers only after the full frame processing and pose estimation.
    prevFramePts = nextFramePoints.map((p) => ({ x: p.x, y: p.y }));
    replacePrevGray(gray);

    trackedData = {
      loading: false,
      trackedCount,
      trackedPoints: pointsToFlatArray(nextFramePoints),
      prevPoints: pointsToFlatArray(prevMatchedPoints),
      status: statusText,
      homography: {
        H: lastHomography,
        inliers: homographyEstimate.inliers,
        confidence: homographyEstimate.confidence,
        status: homographyEstimate.status
      },
      pose: selectedPose,
      poseEssential: essentialPose,
      poseDebug: {
        selectedSource,
        confidence: selected.confidence,
        reprojectionError,
        wasClamped
      }
    };
  } catch (cvError) {
    console.error('Frame tracking error:', cvError);
  }

  rgba.delete();
  gray.delete();

  return trackedData;
}

export { initTracking, processTrackingFrame };
