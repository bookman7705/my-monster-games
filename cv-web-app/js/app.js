import { initCamera, getVideo } from './camera.js';
import { initTracking, processTrackingFrame } from './tracking.js?v=tracking-fix-1';
import { drawTracking } from './visualization.js';
import {
  initUI,
  updateHUD,
  isDebugDrawEnabled,
  isHomographyDebugEnabled,
  consumeRedetectRequest
} from './ui.js';

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

let loopStarted = false;

function startRenderLoop() {
  if (loopStarted) {
    return;
  }
  loopStarted = true;
  processFrame();
}

function processFrame() {
  const video = getVideo();
  if (!video) {
    requestAnimationFrame(processFrame);
    return;
  }

  if (video.readyState === video.HAVE_ENOUGH_DATA) {
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;

    ctx.drawImage(video, 0, 0);

    const trackingData = processTrackingFrame(canvas, {
      forceRedetect: consumeRedetectRequest()
    });

    drawTracking(
      ctx,
      trackingData,
      isDebugDrawEnabled(),
      isHomographyDebugEnabled()
    );
    updateHUD(trackingData.trackedCount, trackingData.status);
  }

  requestAnimationFrame(processFrame);
}

async function initApp() {
  initUI();
  await initCamera();
  await initTracking(getVideo());

  const video = getVideo();
  if (!video) {
    return;
  }

  // Mobile browsers can fire play before listeners are attached, so we use
  // multiple readiness hooks and an immediate readyState check.
  video.addEventListener('play', startRenderLoop);
  video.addEventListener('playing', startRenderLoop);
  video.addEventListener('loadeddata', startRenderLoop);

  if (video.readyState >= video.HAVE_CURRENT_DATA) {
    startRenderLoop();
  }
}

initApp().catch((error) => {
  console.error('App initialization failed:', error);
});
