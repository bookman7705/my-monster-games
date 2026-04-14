const video = document.getElementById('video');
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

// ---- SLAM tracking state ----
let cvReady = false;
let prevGray = null;
let prevPts = null;
let activeTrackedPoints = 0;
let debugDrawEnabled = true;
let forceRedetectRequested = false;

const MAX_CORNERS = 100;
const QUALITY_LEVEL = 0.01;
const MIN_DISTANCE = 10;
const LOST_THRESHOLD = 20;
const OK_THRESHOLD = 50;

// Lightweight debug UI (kept in JS so only app.js changes are needed)
const hud = document.createElement('div');
const counterText = document.createElement('div');
const trackingText = document.createElement('div');
const helpText = document.createElement('div');
const controls = document.createElement('div');
const debugToggleBtn = document.createElement('button');
const redetectBtn = document.createElement('button');

hud.style.position = 'fixed';
hud.style.top = '12px';
hud.style.left = '12px';
hud.style.background = 'rgba(0, 0, 0, 0.55)';
hud.style.color = '#fff';
hud.style.padding = '8px 10px';
hud.style.borderRadius = '8px';
hud.style.fontFamily = 'sans-serif';
hud.style.fontSize = '14px';
hud.style.textAlign = 'left';
hud.style.zIndex = '9999';
hud.style.pointerEvents = 'none';

helpText.style.opacity = '0.8';
helpText.style.fontSize = '12px';
helpText.textContent = 'Use the buttons below for debug controls';

controls.style.position = 'fixed';
controls.style.left = '12px';
controls.style.bottom = '12px';
controls.style.display = 'flex';
controls.style.flexDirection = 'column';
controls.style.gap = '8px';
controls.style.zIndex = '9999';
controls.style.pointerEvents = 'auto';

debugToggleBtn.style.padding = '10px 12px';
debugToggleBtn.style.border = '1px solid #fff';
debugToggleBtn.style.borderRadius = '8px';
debugToggleBtn.style.background = 'rgba(0, 0, 0, 0.65)';
debugToggleBtn.style.color = '#fff';
debugToggleBtn.style.fontSize = '14px';
debugToggleBtn.style.cursor = 'pointer';
debugToggleBtn.style.touchAction = 'manipulation';

redetectBtn.style.padding = '10px 12px';
redetectBtn.style.border = '1px solid #fff';
redetectBtn.style.borderRadius = '8px';
redetectBtn.style.background = 'rgba(0, 0, 0, 0.65)';
redetectBtn.style.color = '#fff';
redetectBtn.style.fontSize = '14px';
redetectBtn.style.cursor = 'pointer';
redetectBtn.style.touchAction = 'manipulation';

function updateDebugButtonLabel() {
  debugToggleBtn.textContent = debugDrawEnabled ? 'Debug Draw: ON' : 'Debug Draw: OFF';
}

function toggleDebugDrawing() {
  debugDrawEnabled = !debugDrawEnabled;
  updateDebugButtonLabel();
  console.log(`Debug drawing ${debugDrawEnabled ? 'enabled' : 'disabled'}`);
}

debugToggleBtn.addEventListener('click', toggleDebugDrawing);

redetectBtn.textContent = 'Re-detect Features';
redetectBtn.addEventListener('click', () => {
  // Defers reset to the frame loop so OpenCV state changes stay synchronized.
  forceRedetectRequested = true;
});

updateDebugButtonLabel();
controls.appendChild(debugToggleBtn);
controls.appendChild(redetectBtn);

hud.appendChild(counterText);
hud.appendChild(trackingText);
hud.appendChild(helpText);
document.body.appendChild(hud);
document.body.appendChild(controls);

function updateHud(points) {
  activeTrackedPoints = points;
  counterText.textContent = `Tracked points: ${activeTrackedPoints}`;

  if (activeTrackedPoints < LOST_THRESHOLD) {
    trackingText.textContent = 'Tracking LOST';
    trackingText.style.color = '#ff6b6b';
  } else if (activeTrackedPoints > OK_THRESHOLD) {
    trackingText.textContent = 'Tracking OK';
    trackingText.style.color = '#6bff8c';
  } else {
    trackingText.textContent = 'Tracking recovering...';
    trackingText.style.color = '#ffd76b';
  }
}

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

function replacePrevGray(currentGray) {
  if (prevGray) {
    prevGray.delete();
  }
  // Keep an owned copy for the next optical flow step.
  prevGray = currentGray.clone();
}

document.addEventListener('keydown', (event) => {
  if (event.key && event.key.toLowerCase() === 'd') {
    toggleDebugDrawing();
  }
});

// Start camera
async function startCamera() {
  const stream = await navigator.mediaDevices.getUserMedia({
    video: { facingMode: "environment" },
    audio: false
  });
  video.srcObject = stream;

  try {
    await loadOpenCv();
    console.log('OpenCV.js ready');
  } catch (error) {
    console.error(error);
  }
}

startCamera();

// Process frames
function processFrame() {
  if (video.readyState === video.HAVE_ENOUGH_DATA) {
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;

    ctx.drawImage(video, 0, 0);

    if (!cvReady || !window.cv) {
      ctx.fillStyle = '#fff';
      ctx.font = '18px sans-serif';
      ctx.fillText('Loading OpenCV...', 12, 28);
      requestAnimationFrame(processFrame);
      return;
    }

    // Convert rendered canvas frame to Mat so OpenCV can operate on it.
    const rgba = cv.imread(canvas);
    const gray = new cv.Mat();
    cv.cvtColor(rgba, gray, cv.COLOR_RGBA2GRAY);

    let pointsToUse = prevPts;
    let redetectedThisFrame = false;

    if (forceRedetectRequested) {
      if (prevPts) {
        prevPts.delete();
        prevPts = null;
      }
      if (prevGray) {
        prevGray.delete();
        prevGray = null;
      }
      forceRedetectRequested = false;
      console.log('Manual feature re-detection requested');
    }

    // First frame or recovery path: detect strong corners to bootstrap tracking.
    if (!prevGray || !prevPts || prevPts.rows < LOST_THRESHOLD) {
      if (prevPts) {
        prevPts.delete();
      }
      pointsToUse = detectFeatures(gray);
      prevPts = pointsToUse;
      redetectedThisFrame = true;
      console.log(`Re-detected features: ${prevPts.rows}`);
    }

    let trackedCount = 0;
    let trackedPoints = [];
    let prevPoints = [];

    if (!redetectedThisFrame && prevGray && pointsToUse && pointsToUse.rows > 0) {
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

      for (let i = 0; i < status.rows; i++) {
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
      winSize.delete();
      criteria.delete();
    } else if (pointsToUse && pointsToUse.rows > 0) {
      trackedCount = pointsToUse.rows;
      trackedPoints = Array.from(pointsToUse.data32F);
    }

    // Draw tracked landmarks and motion vectors for visual verification.
    if (debugDrawEnabled) {
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#ff4d4d'; // red motion vectors
      ctx.fillStyle = '#4dff4d'; // green tracked points

      if (prevPoints.length > 0) {
        for (let i = 0; i < trackedCount; i++) {
          const px = prevPoints[i * 2];
          const py = prevPoints[i * 2 + 1];
          const cx = trackedPoints[i * 2];
          const cy = trackedPoints[i * 2 + 1];

          ctx.beginPath();
          ctx.moveTo(px, py);
          ctx.lineTo(cx, cy);
          ctx.stroke();

          ctx.beginPath();
          ctx.arc(cx, cy, 3, 0, Math.PI * 2);
          ctx.fill();
        }
      } else {
        for (let i = 0; i < trackedCount; i++) {
          const cx = trackedPoints[i * 2];
          const cy = trackedPoints[i * 2 + 1];
          ctx.beginPath();
          ctx.arc(cx, cy, 3, 0, Math.PI * 2);
          ctx.fill();
        }
      }
    }

    updateHud(trackedCount);

    // If too few tracks survive, refresh features so tracking can recover.
    if (trackedCount < LOST_THRESHOLD) {
      if (prevPts) {
        prevPts.delete();
      }
      prevPts = detectFeatures(gray);
      updateHud(prevPts.rows);
      console.log(`Re-detected features: ${prevPts.rows}`);
    }

    replacePrevGray(gray);

    rgba.delete();
    gray.delete();
  }

  requestAnimationFrame(processFrame);
}

video.addEventListener('play', () => {
  processFrame();
});
