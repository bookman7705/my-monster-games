let debugDrawEnabled = true;
let redetectRequested = false;

let counterText = null;
let trackingText = null;
let debugToggleBtn = null;

function updateDebugButtonLabel() {
  if (!debugToggleBtn) {
    return;
  }
  debugToggleBtn.textContent = debugDrawEnabled ? 'Debug Draw: ON' : 'Debug Draw: OFF';
}

function toggleDebugDrawing() {
  debugDrawEnabled = !debugDrawEnabled;
  updateDebugButtonLabel();
  console.log(`Debug drawing ${debugDrawEnabled ? 'enabled' : 'disabled'}`);
}

function initUI() {
  const hud = document.createElement('div');
  counterText = document.createElement('div');
  trackingText = document.createElement('div');
  const helpText = document.createElement('div');
  const controls = document.createElement('div');
  debugToggleBtn = document.createElement('button');
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

  debugToggleBtn.addEventListener('click', toggleDebugDrawing);
  redetectBtn.textContent = 'Re-detect Features';
  redetectBtn.addEventListener('click', () => {
    // Defers reset to the frame loop so OpenCV state changes stay synchronized.
    redetectRequested = true;
  });

  document.addEventListener('keydown', (event) => {
    if (event.key && event.key.toLowerCase() === 'd') {
      toggleDebugDrawing();
    }
  });

  updateDebugButtonLabel();
  controls.appendChild(debugToggleBtn);
  controls.appendChild(redetectBtn);

  hud.appendChild(counterText);
  hud.appendChild(trackingText);
  hud.appendChild(helpText);

  document.body.appendChild(hud);
  document.body.appendChild(controls);
}

function updateHUD(trackedPoints, status) {
  if (!counterText || !trackingText) {
    return;
  }

  counterText.textContent = `Tracked points: ${trackedPoints}`;
  trackingText.textContent = status;

  if (status === 'Tracking LOST') {
    trackingText.style.color = '#ff6b6b';
  } else if (status === 'Tracking OK') {
    trackingText.style.color = '#6bff8c';
  } else {
    trackingText.style.color = '#ffd76b';
  }
}

function isDebugDrawEnabled() {
  return debugDrawEnabled;
}

function consumeRedetectRequest() {
  const requested = redetectRequested;
  redetectRequested = false;
  return requested;
}

export { initUI, updateHUD, isDebugDrawEnabled, consumeRedetectRequest };
