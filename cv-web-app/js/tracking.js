let videoRef = null;
let cvReady = false;
let prevGray = null;
let prevPts = null;

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
      status: 'Tracking LOST'
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
    status: 'Tracking LOST'
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

    trackedData = {
      loading: false,
      trackedCount,
      trackedPoints,
      prevPoints,
      status: statusText
    };
  } catch (cvError) {
    console.error('Frame tracking error:', cvError);
  }

  rgba.delete();
  gray.delete();

  return trackedData;
}

export { initTracking, processTrackingFrame };
