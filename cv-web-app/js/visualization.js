function drawTracking(ctx, trackingData, debugEnabled) {
  if (!trackingData) {
    return;
  }

  if (trackingData.loading) {
    ctx.fillStyle = '#fff';
    ctx.font = '18px sans-serif';
    ctx.fillText('Loading OpenCV...', 12, 28);
    return;
  }

  if (!debugEnabled) {
    return;
  }

  const trackedCount = trackingData.trackedCount || 0;
  const trackedPoints = trackingData.trackedPoints || [];
  const prevPoints = trackingData.prevPoints || [];

  // Draw tracked landmarks and motion vectors for visual verification.
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
      ctx.arc(cx, cy, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#111';
      ctx.stroke();
      ctx.strokeStyle = '#ff4d4d';
    }
  } else {
    for (let i = 0; i < trackedCount; i++) {
      const cx = trackedPoints[i * 2];
      const cy = trackedPoints[i * 2 + 1];
      ctx.beginPath();
      ctx.arc(cx, cy, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#111';
      ctx.stroke();
      ctx.strokeStyle = '#ff4d4d';
    }
  }
}

export { drawTracking };
