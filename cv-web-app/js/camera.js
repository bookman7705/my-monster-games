let videoRef = null;
let streamRef = null;

// Initializes mobile-friendly rear camera capture for AR/SLAM input.
async function initCamera() {
  videoRef = document.getElementById('video');
  if (!videoRef) {
    throw new Error('Video element #video not found');
  }

  streamRef = await navigator.mediaDevices.getUserMedia({
    video: { facingMode: 'environment' },
    audio: false
  });

  videoRef.srcObject = streamRef;
  return streamRef;
}

function getVideo() {
  return videoRef;
}

export { initCamera, getVideo };
