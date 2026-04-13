const video = document.getElementById('video');
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

// Start camera
async function startCamera() {
  const stream = await navigator.mediaDevices.getUserMedia({
    video: { facingMode: "environment" },
    audio: false
  });
  video.srcObject = stream;
}

startCamera();

// Process frames
function processFrame() {
  if (video.readyState === video.HAVE_ENOUGH_DATA) {
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;

    ctx.drawImage(video, 0, 0);

    let frame = ctx.getImageData(0, 0, canvas.width, canvas.height);
    let data = frame.data;

    // Grayscale + simple edge detection
    for (let i = 0; i < data.length; i += 4) {
      let r = data[i];
      let g = data[i + 1];
      let b = data[i + 2];

      let gray = 0.3 * r + 0.59 * g + 0.11 * b;

      // Simple threshold edge effect
      let edge = gray > 100 ? 255 : 0;

      data[i] = data[i + 1] = data[i + 2] = edge;
    }

    ctx.putImageData(frame, 0, 0);
  }

  requestAnimationFrame(processFrame);
}

video.addEventListener('play', () => {
  processFrame();
});
