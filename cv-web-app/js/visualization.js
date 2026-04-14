function isHomographyDebugEnabled() {
  return window.__homographyDebugEnabled !== false;
}

function getHomographyOverlayColor(homography) {
  if (!homography || !homography.H) {
    return 'rgba(100,100,100,0.1)';
  }
  if (homography.status === 'stable') {
    return 'rgba(0,255,0,0.12)';
  }
  if (homography.status === 'medium') {
    return 'rgba(255,255,0,0.12)';
  }
  return 'rgba(255,0,0,0.12)';
}

function drawHomographyBackground(ctx, trackingData) {
  const overlayColor = getHomographyOverlayColor(trackingData.homography);
  ctx.fillStyle = overlayColor;
  ctx.fillRect(0, 0, ctx.canvas.width, ctx.canvas.height);
}

function getHomographyTranslationVector(homography) {
  if (!homography || !homography.H) {
    return null;
  }

  const matrixData = homography.H.data64F || homography.H.data32F;
  if (!matrixData || matrixData.length < 9) {
    return null;
  }

  // Translation terms in 3x3 homography matrix: [h13, h23].
  return {
    x: matrixData[2],
    y: matrixData[5]
  };
}

function getCentroidMotionVector(trackingData) {
  const trackedPoints = trackingData.trackedPoints || [];
  const prevPoints = trackingData.prevPoints || [];
  const total = Math.min(
    Math.floor(trackedPoints.length / 2),
    Math.floor(prevPoints.length / 2),
    100
  );
  if (total === 0) {
    return null;
  }

  let dxSum = 0;
  let dySum = 0;
  for (let i = 0; i < total; i++) {
    const px = prevPoints[i * 2];
    const py = prevPoints[i * 2 + 1];
    const cx = trackedPoints[i * 2];
    const cy = trackedPoints[i * 2 + 1];
    dxSum += cx - px;
    dySum += cy - py;
  }

  return {
    x: dxSum / total,
    y: dySum / total
  };
}

function drawMotionArrow(ctx, trackingData) {
  const homography = trackingData.homography;
  if (!homography || !homography.H) {
    return;
  }

  let vector = getHomographyTranslationVector(homography);
  if (!vector) {
    vector = getCentroidMotionVector(trackingData);
  }
  if (!vector) {
    return;
  }

  const centerX = ctx.canvas.width / 2;
  const centerY = ctx.canvas.height / 2;
  const scale = 4;
  const maxLength = 120;

  let dx = vector.x * scale;
  let dy = vector.y * scale;
  const length = Math.hypot(dx, dy);
  if (length > maxLength && length > 0) {
    const factor = maxLength / length;
    dx *= factor;
    dy *= factor;
  }

  const endX = centerX + dx;
  const endY = centerY + dy;

  ctx.strokeStyle = '#00e5ff';
  ctx.fillStyle = '#00e5ff';
  ctx.lineWidth = 3;

  ctx.beginPath();
  ctx.moveTo(centerX, centerY);
  ctx.lineTo(endX, endY);
  ctx.stroke();

  const angle = Math.atan2(dy, dx);
  const headLen = 10;
  ctx.beginPath();
  ctx.moveTo(endX, endY);
  ctx.lineTo(
    endX - headLen * Math.cos(angle - Math.PI / 6),
    endY - headLen * Math.sin(angle - Math.PI / 6)
  );
  ctx.lineTo(
    endX - headLen * Math.cos(angle + Math.PI / 6),
    endY - headLen * Math.sin(angle + Math.PI / 6)
  );
  ctx.closePath();
  ctx.fill();
}

function drawHomographyInfoPanel(ctx, trackingData) {
  const homography = trackingData.homography || {};
  const status = (homography.status || 'unstable').toUpperCase();
  const confidenceValue = Number.isFinite(homography.confidence)
    ? homography.confidence
    : 0;
  const confidence = confidenceValue.toFixed(2);
  const inliers = Number.isFinite(homography.inliers) ? homography.inliers : 0;
  const points = Number.isFinite(trackingData.trackedCount) ? trackingData.trackedCount : 0;

  const panelX = 12;
  const panelY = 74;
  const panelWidth = 220;
  const panelHeight = 124;

  ctx.fillStyle = 'rgba(0, 0, 0, 0.55)';
  ctx.fillRect(panelX, panelY, panelWidth, panelHeight);

  ctx.fillStyle = '#ffffff';
  ctx.font = '13px monospace';
  ctx.fillText('HOMOGRAPHY DEBUG', panelX + 10, panelY + 20);
  ctx.fillText('----------------', panelX + 10, panelY + 38);
  ctx.fillText(`Points: ${points}`, panelX + 10, panelY + 56);
  ctx.fillText(`Inliers: ${inliers}`, panelX + 10, panelY + 74);
  ctx.fillText(`Confidence: ${confidence}`, panelX + 10, panelY + 92);
  ctx.fillText(`Status: ${status}`, panelX + 10, panelY + 110);
}

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

  if (isHomographyDebugEnabled()) {
    drawHomographyBackground(ctx, trackingData);
    drawHomographyInfoPanel(ctx, trackingData);
    drawMotionArrow(ctx, trackingData);
  }

  if (!debugEnabled) {
    return;
  }

  const trackedCount = trackingData.trackedCount || 0;
  const trackedPoints = trackingData.trackedPoints || [];
  const prevPoints = trackingData.prevPoints || [];
  const maxPoints = Math.min(100, trackedCount);

  // Draw tracked landmarks and motion vectors for visual verification.
  ctx.lineWidth = 2;
  ctx.strokeStyle = '#ff4d4d'; // red motion vectors
  ctx.fillStyle = '#4dff4d'; // green tracked points

  if (prevPoints.length > 0) {
    for (let i = 0; i < maxPoints; i++) {
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
    for (let i = 0; i < maxPoints; i++) {
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
