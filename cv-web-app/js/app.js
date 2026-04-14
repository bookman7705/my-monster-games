import { initCamera, getVideo } from './camera.js';
import { initTracking, processTrackingFrame } from './tracking.js';
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

  video.addEventListener('play', () => {
    if (loopStarted) {
      return;
    }
    loopStarted = true;
    processFrame();
  });
}

initApp().catch((error) => {
  console.error('App initialization failed:', error);
});
