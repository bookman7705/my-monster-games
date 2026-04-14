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

function buildIntrinsicMatrix(canvasWidth, canvasHeight) {
  const fx = canvasWidth;
  const fy = canvasWidth;
  const cx = canvasWidth / 2;
  const cy = canvasHeight / 2;
  return [
    [fx, 0, cx],
    [0, fy, cy],
    [0, 0, 1]
  ];
}

function multiplyMat3Vec3(mat3, vec3) {
  return [
    mat3[0][0] * vec3[0] + mat3[0][1] * vec3[1] + mat3[0][2] * vec3[2],
    mat3[1][0] * vec3[0] + mat3[1][1] * vec3[1] + mat3[1][2] * vec3[2],
    mat3[2][0] * vec3[0] + mat3[2][1] * vec3[1] + mat3[2][2] * vec3[2]
  ];
}

function addVec3(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function projectPoint(point3, pose, K) {
  if (!pose || !pose.R || !pose.t || pose.R.length !== 3 || pose.t.length !== 3) {
    return null;
  }

  const rotated = multiplyMat3Vec3(pose.R, point3);
  const cameraPoint = addVec3(rotated, pose.t);
  const projected = multiplyMat3Vec3(K, cameraPoint);
  if (Math.abs(projected[2]) < 1e-6) {
    return null;
  }

  return {
    x: projected[0] / projected[2],
    y: projected[1] / projected[2]
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

function drawPoseMotionArrow(ctx, pose) {
  if (!pose || !pose.valid || !pose.t || pose.t.length !== 3) {
    return;
  }

  const centerX = ctx.canvas.width / 2;
  const centerY = ctx.canvas.height / 2;
  const scale = 120;
  const maxLength = 120;

  let dx = pose.t[0] * scale;
  let dy = pose.t[1] * scale;
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

function drawWorldAxes(ctx, pose) {
  if (!pose || !pose.valid) {
    return;
  }

  const centerX = ctx.canvas.width / 2;
  const centerY = ctx.canvas.height / 2;
  const K = buildIntrinsicMatrix(ctx.canvas.width, ctx.canvas.height);
  const axisScale = 1;
  const axisLengthPixels = 100;

  const projectedOrigin = projectPoint([0, 0, 0], pose, K);
  const projectedX = projectPoint([axisScale, 0, 0], pose, K);
  const projectedY = projectPoint([0, axisScale, 0], pose, K);
  const projectedZ = projectPoint([0, 0, axisScale], pose, K);

  if (!projectedOrigin || !projectedX || !projectedY || !projectedZ) {
    return;
  }

  const axisDefinitions = [
    { projected: projectedX, color: '#ff3b30' }, // X red
    { projected: projectedY, color: '#34c759' }, // Y green
    { projected: projectedZ, color: '#007aff' } // Z blue
  ];

  for (let i = 0; i < axisDefinitions.length; i++) {
    const axis = axisDefinitions[i];
    let dx = axis.projected.x - projectedOrigin.x;
    let dy = axis.projected.y - projectedOrigin.y;
    const length = Math.hypot(dx, dy);
    if (length > 0) {
      dx = (dx / length) * axisLengthPixels;
      dy = (dy / length) * axisLengthPixels;
    }

    ctx.strokeStyle = axis.color;
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX + dx, centerY + dy);
    ctx.stroke();
  }

  // Stable world origin marker anchored at screen center.
  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.arc(centerX, centerY, 6, 0, Math.PI * 2);
  ctx.fill();
}

function getApproxRotationDegrees(R) {
  if (!R || R.length !== 3) {
    return { rx: 0, ry: 0, rz: 0 };
  }
  const rx = Math.atan2(R[2][1], R[2][2]) * (180 / Math.PI);
  const ry = Math.atan2(-R[2][0], Math.sqrt((R[2][1] ** 2) + (R[2][2] ** 2))) * (180 / Math.PI);
  const rz = Math.atan2(R[1][0], R[0][0]) * (180 / Math.PI);
  return { rx, ry, rz };
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

function drawPoseInfoPanel(ctx, trackingData) {
  const pose = trackingData.pose || { R: [], t: [], valid: false };
  const t = pose.t || [];
  const tx = Number.isFinite(t[0]) ? t[0] : 0;
  const ty = Number.isFinite(t[1]) ? t[1] : 0;
  const tz = Number.isFinite(t[2]) ? t[2] : 0;
  const rot = getApproxRotationDegrees(pose.R);

  const panelX = 12;
  const panelY = 206;
  const panelWidth = 250;
  const panelHeight = 138;

  ctx.fillStyle = 'rgba(0, 0, 0, 0.55)';
  ctx.fillRect(panelX, panelY, panelWidth, panelHeight);

  ctx.font = '13px monospace';
  ctx.fillStyle = '#ffffff';
  ctx.fillText('POSE DEBUG', panelX + 10, panelY + 20);
  ctx.fillText('----------', panelX + 10, panelY + 38);
  ctx.fillStyle = pose.valid ? '#6bff8c' : '#ff6b6b';
  ctx.fillText(`Pose: ${pose.valid ? 'VALID' : 'INVALID'}`, panelX + 10, panelY + 56);
  ctx.fillStyle = '#ffffff';
  ctx.fillText(`Tx: ${tx.toFixed(3)}`, panelX + 10, panelY + 74);
  ctx.fillText(`Ty: ${ty.toFixed(3)}`, panelX + 10, panelY + 92);
  ctx.fillText(`Tz: ${tz.toFixed(3)}`, panelX + 10, panelY + 110);
  ctx.fillText(`Rot: ${rot.rx.toFixed(1)}, ${rot.ry.toFixed(1)}, ${rot.rz.toFixed(1)}`, panelX + 10, panelY + 128);
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
    drawPoseInfoPanel(ctx, trackingData);
    const pose = trackingData.pose || { valid: false };
    if (pose.valid) {
      drawWorldAxes(ctx, pose);
      drawPoseMotionArrow(ctx, pose);
    } else {
      ctx.fillStyle = '#ff6b6b';
      ctx.font = '16px sans-serif';
      ctx.fillText('Pose invalid', 12, 362);
    }
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
