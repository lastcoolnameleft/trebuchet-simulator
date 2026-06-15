const PARAMS = [
  { id: 'LAl', label: 'Arm length (long)', unit: 'm', step: 0.1, min: 1, max: 20, default: 6 },
  { id: 'LAs', label: 'Arm length (short)', unit: 'm', step: 0.1, min: 0.5, max: 10, default: 2 },
  { id: 'LAcg', label: 'Arm CG from pivot', unit: 'm', step: 0.1, min: -10, max: 10, default: -2, derived: true },
  { id: 'LW', label: 'CW hanging length', unit: 'm', step: 0.1, min: 0.25, max: 10, default: 2 },
  { id: 'LS', label: 'Sling length', unit: 'm', step: 0.1, min: 1, max: 20, default: 6 },
  { id: 'h', label: 'Pivot height', unit: 'm', step: 0.1, min: 1, max: 20, default: 5 },
  { id: 'mA', label: 'Arm mass', unit: 'kg', step: 1, min: 1, max: 2000, default: 50 },
  { id: 'mW', label: 'Counterweight mass', unit: 'kg', step: 1, min: 1, max: 10000, default: 200 },
  { id: 'mP', label: 'Projectile mass', unit: 'kg', step: 0.1, min: 0.1, max: 500, default: 10 },
  { id: 'IA3', label: 'Arm inertia', unit: 'kg·m²', step: 1, min: 0.1, max: 50000, default: 266.67, derived: true },
  { id: 'IW3', label: 'CW inertia', unit: 'kg·m²', step: 1, min: 0.1, max: 50000, default: 66.67, derived: true },
  { id: 'releaseAngle', label: 'Release angle', unit: '°', step: 1, min: 5, max: 85, default: 45 },
  { id: 'projectileDiameter', label: 'Projectile diameter', unit: 'm', step: 0.01, min: 0.01, max: 2, default: 0.18 },
  { id: 'dragCoefficient', label: 'Drag coefficient', unit: '', step: 0.01, min: 0, max: 2, default: 0.47 },
  { id: 'airDensity', label: 'Air density', unit: 'kg/m³', step: 0.01, min: 0.1, max: 5, default: 1.225 },
  { id: 'windSpeed', label: 'Wind speed', unit: 'm/s', step: 0.1, min: -50, max: 50, default: 0 },
];

const DEFAULTS = Object.fromEntries(PARAMS.map((field) => [field.id, field.default]));

const appState = {
  pyodide: null,
  simulateFn: null,
  ready: false,
  loading: true,
  simulation: null,
  animationHandle: null,
  animationStart: null,
  viewport: null,
};

const ui = {
  form: document.getElementById('parameterForm'),
  fireButton: document.getElementById('fireButton'),
  resetButton: document.getElementById('resetButton'),
  autoDerived: document.getElementById('autoDerived'),
  enableAirDrag: document.getElementById('enableAirDrag'),
  status: document.getElementById('statusText'),
  loading: document.getElementById('loadingOverlay'),
  stats: document.getElementById('statsGrid'),
  canvas: document.getElementById('simCanvas'),
  ctx: document.getElementById('simCanvas').getContext('2d'),
};

const STAT_FIELDS = [
  ['range', 'Throw distance', 'm'],
  ['landingX', 'Landing x from pivot', 'm'],
  ['maxHeight', 'Peak height', 'm'],
  ['releaseSpeed', 'Release speed', 'm/s'],
  ['releaseAngleActual', 'Actual release angle', '°'],
  ['liftOffTime', 'Lift-off time', 's'],
  ['releaseTime', 'Release time', 's'],
  ['flightTime', 'Flight time', 's'],
  ['totalTime', 'Total simulation time', 's'],
];

function buildForm() {
  ui.form.innerHTML = PARAMS.map((field) => `
    <label class="field">
      <span>${field.label}</span>
      <div class="field-input">
        <input id="param-${field.id}" type="number" step="${field.step}" min="${field.min}" max="${field.max}" value="${field.default}">
        <span class="unit">${field.unit}</span>
      </div>
    </label>
  `).join('');

  for (const field of PARAMS) {
    const input = document.getElementById(`param-${field.id}`);
    input.addEventListener('input', () => {
      clearSimulationState();
      if (ui.autoDerived.checked) {
        syncDerivedValues();
      }
      drawIdlePose();
    });
  }

  ui.autoDerived.addEventListener('change', () => {
    clearSimulationState();
    syncDerivedValues();
    updateDerivedInputState();
    drawIdlePose();
  });

  ui.enableAirDrag.addEventListener('change', () => {
    clearSimulationState();
    drawIdlePose();
  });

  syncDerivedValues();
  updateDerivedInputState();
}

function updateDerivedInputState() {
  for (const field of PARAMS.filter((entry) => entry.derived)) {
    document.getElementById(`param-${field.id}`).disabled = ui.autoDerived.checked;
  }
}

function syncDerivedValues() {
  const values = readRawValues();
  const LAcg = (values.LAs - values.LAl) / 2;
  const IA3 = values.mA * (values.LAl + values.LAs) ** 2 / 12;
  const IW3 = values.mW * values.LW ** 2 / 12;

  if (ui.autoDerived.checked) {
    document.getElementById('param-LAcg').value = LAcg.toFixed(3);
    document.getElementById('param-IA3').value = IA3.toFixed(3);
    document.getElementById('param-IW3').value = IW3.toFixed(3);
  }
}

function readRawValues() {
  return Object.fromEntries(
    PARAMS.map((field) => [field.id, Number.parseFloat(document.getElementById(`param-${field.id}`).value)])
  );
}

function getParams() {
  const values = readRawValues();
  return {
    ...values,
    autoDerived: ui.autoDerived.checked,
    enableAirDrag: ui.enableAirDrag.checked,
  };
}

function renderStats(stats) {
  if (!stats) {
    ui.stats.innerHTML = '<div class="stats-placeholder">Press Fire once Pyodide is ready.</div>';
    return;
  }

  ui.stats.innerHTML = STAT_FIELDS.map(([key, label, unit]) => `
    <div class="stat-card">
      <span class="stat-label">${label}</span>
      <strong>${Number(stats[key]).toFixed(2)}${unit ? ` <small>${unit}</small>` : ''}</strong>
    </div>
  `).join('');
}

function setStatus(message, isError = false) {
  ui.status.textContent = message;
  ui.status.dataset.state = isError ? 'error' : 'ok';
}

function showLoading(message) {
  ui.loading.hidden = false;
  ui.loading.querySelector('strong').textContent = message;
}

function hideLoading() {
  ui.loading.hidden = true;
}

function stopAnimation() {
  if (appState.animationHandle !== null) {
    cancelAnimationFrame(appState.animationHandle);
    appState.animationHandle = null;
  }
  appState.animationStart = null;
}

function clearSimulationState() {
  stopAnimation();
  appState.simulation = null;
  renderStats(null);
  if (appState.ready) {
    setStatus('Inputs updated. Press Fire to launch.');
  }
}

function computeGroundSq(params) {
  const Aq0 = Math.PI / 4;
  const cosTotal = (params.h - params.LAl * Math.cos(Aq0)) / params.LS;
  if (cosTotal < -1 || cosTotal > 1) {
    return null;
  }
  return Math.acos(cosTotal) - Aq0;
}

function computeViewport(frames) {
  const xs = [0, ...frames.armTipX, ...frames.cwAttachX, ...frames.weightX, ...frames.projectileX];
  const ys = [0, ...frames.armTipY, ...frames.cwAttachY, ...frames.weightY, ...frames.projectileY];
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const maxY = Math.max(...ys, 0);
  return { minX, maxX, maxY };
}

function worldToCanvas(x, y, viewport) {
  const { width, height } = ui.canvas;
  const leftRoom = width * 0.35;
  const rightRoom = width * 0.55;
  const topPad = 24;
  const bottomPad = 48;

  const leftExtent = Math.max(Math.abs(viewport.minX), 1);
  const rightExtent = Math.max(viewport.maxX, 1);
  const worldHeight = Math.max(viewport.maxY, 1);
  const scale = Math.min(
    (leftRoom - 24) / leftExtent,
    (rightRoom - 24) / rightExtent,
    (height - topPad - bottomPad) / worldHeight
  );

  const pivotX = Math.max(120, Math.min(width - 120, leftRoom));
  const groundY = height - bottomPad;
  return {
    x: pivotX + x * scale,
    y: groundY - y * scale,
    scale,
    pivotX,
    groundY,
  };
}

function drawGround(viewport, landingX = null, range = null) {
  const { ctx } = ui;
  const origin = worldToCanvas(0, 0, viewport);

  ctx.save();
  ctx.strokeStyle = '#c08a42';
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(0, origin.groundY);
  ctx.lineTo(ui.canvas.width, origin.groundY);
  ctx.stroke();

  if (landingX !== null) {
    const impact = worldToCanvas(landingX, 0, viewport);
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(impact.x, origin.groundY - 18);
    ctx.lineTo(impact.x, origin.groundY + 4);
    ctx.stroke();

    ctx.fillStyle = '#f8fafc';
    ctx.font = '12px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(`${range.toFixed(1)} m`, (origin.pivotX + impact.x) / 2, origin.groundY - 10);

    ctx.setLineDash([6, 4]);
    ctx.strokeStyle = 'rgba(248, 250, 252, 0.5)';
    ctx.beginPath();
    ctx.moveTo(origin.pivotX, origin.groundY - 14);
    ctx.lineTo(impact.x, origin.groundY - 14);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  ctx.restore();
}

function drawTrail(index, sim, viewport) {
  const { ctx } = ui;
  const start = sim.releaseIndex;
  if (index <= start) return;

  ctx.save();
  ctx.setLineDash([4, 6]);
  ctx.strokeStyle = 'rgba(248, 250, 252, 0.55)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  for (let i = start; i <= index; i += 1) {
    const point = worldToCanvas(sim.frames.projectileX[i], sim.frames.projectileY[i], viewport);
    if (i === start) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  }
  ctx.stroke();
  ctx.restore();
}

function drawFrame(index, sim = appState.simulation) {
  const { ctx } = ui;
  const viewport = appState.viewport;
  ctx.clearRect(0, 0, ui.canvas.width, ui.canvas.height);

  ctx.fillStyle = '#0f172a';
  ctx.fillRect(0, 0, ui.canvas.width, ui.canvas.height);

  drawGround(viewport, sim?.stats?.landingX ?? null, sim?.stats?.range ?? null);

  const pivot = worldToCanvas(sim.frames.pivotX[index], sim.frames.pivotY[index], viewport);
  const armTip = worldToCanvas(sim.frames.armTipX[index], sim.frames.armTipY[index], viewport);
  const cwAttach = worldToCanvas(sim.frames.cwAttachX[index], sim.frames.cwAttachY[index], viewport);
  const weight = worldToCanvas(sim.frames.weightX[index], sim.frames.weightY[index], viewport);
  const projectile = worldToCanvas(sim.frames.projectileX[index], sim.frames.projectileY[index], viewport);

  drawTrail(index, sim, viewport);

  ctx.save();
  ctx.strokeStyle = '#64748b';
  ctx.lineWidth = 8;
  ctx.beginPath();
  ctx.moveTo(pivot.x, pivot.groundY);
  ctx.lineTo(pivot.x, pivot.y);
  ctx.stroke();

  ctx.strokeStyle = '#f59e0b';
  ctx.lineWidth = 6;
  ctx.beginPath();
  ctx.moveTo(cwAttach.x, cwAttach.y);
  ctx.lineTo(armTip.x, armTip.y);
  ctx.stroke();

  if (sim.frames.projectileAttached[index]) {
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(armTip.x, armTip.y);
    ctx.lineTo(projectile.x, projectile.y);
    ctx.stroke();
  }

  ctx.fillStyle = '#f8fafc';
  ctx.beginPath();
  ctx.arc(pivot.x, pivot.y, 6, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = '#fb7185';
  ctx.beginPath();
  ctx.arc(weight.x, weight.y, 12, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = '#38bdf8';
  ctx.beginPath();
  ctx.arc(projectile.x, projectile.y, 7, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawIdlePose() {
  if (appState.simulation) {
    drawFrame(0);
    return;
  }

  const params = getParams();
  const Sq = computeGroundSq(params);
  const ctx = ui.ctx;
  ctx.clearRect(0, 0, ui.canvas.width, ui.canvas.height);
  ctx.fillStyle = '#0f172a';
  ctx.fillRect(0, 0, ui.canvas.width, ui.canvas.height);

  if (Sq === null) {
    setStatus('Current geometry cannot place the projectile on the ground.', true);
    renderStats(null);
    return;
  }

  const Aq = Math.PI / 4;
  const Wq = 0;
  const viewport = computeViewport({
    armTipX: [-params.LAl * Math.sin(Aq)],
    armTipY: [params.h - params.LAl * Math.cos(Aq)],
    cwAttachX: [params.LAs * Math.sin(Aq)],
    cwAttachY: [params.h + params.LAs * Math.cos(Aq)],
    weightX: [params.LAs * Math.sin(Aq) + params.LW * Math.sin(Aq + Wq)],
    weightY: [params.h + params.LAs * Math.cos(Aq) + params.LW * Math.cos(Aq + Wq)],
    projectileX: [-params.LAl * Math.sin(Aq) - params.LS * Math.sin(Aq + Sq)],
    projectileY: [params.h - (params.LAl * Math.cos(Aq) + params.LS * Math.cos(Aq + Sq))],
  });

  appState.viewport = viewport;
  const sim = {
    frames: {
      pivotX: [0], pivotY: [params.h],
      armTipX: [-params.LAl * Math.sin(Aq)], armTipY: [params.h - params.LAl * Math.cos(Aq)],
      cwAttachX: [params.LAs * Math.sin(Aq)], cwAttachY: [params.h + params.LAs * Math.cos(Aq)],
      weightX: [params.LAs * Math.sin(Aq) + params.LW * Math.sin(Aq + Wq)],
      weightY: [params.h + params.LAs * Math.cos(Aq) + params.LW * Math.cos(Aq + Wq)],
      projectileX: [-params.LAl * Math.sin(Aq) - params.LS * Math.sin(Aq + Sq)],
      projectileY: [params.h - (params.LAl * Math.cos(Aq) + params.LS * Math.cos(Aq + Sq))],
      projectileAttached: [1],
    },
    releaseIndex: 0,
    stats: null,
  };
  drawGround(viewport);
  drawFrame(0, sim);
}

function normalizeSimulation(result) {
  const frames = Object.fromEntries(
    Object.entries(result.frames).map(([key, values]) => [key, key === 'projectileAttached' ? values.map(Boolean) : Float64Array.from(values)])
  );
  return {
    ...result,
    times: Float64Array.from(result.times),
    frames,
  };
}

async function runSimulation() {
  if (!appState.ready) return;

  stopAnimation();
  appState.simulation = null;
  renderStats(null);
  const params = getParams();
  setStatus('Running analytical physics in Pyodide…');
  showLoading('Running simulation…');
  await new Promise((resolve) => requestAnimationFrame(resolve));

  const pyParams = appState.pyodide.toPy(params);
  let pyResult;
  try {
    pyResult = appState.simulateFn(pyParams);
    const result = pyResult.toJs({ dict_converter: Object.fromEntries });
    if (!result.ok) {
      throw new Error(result.error);
    }
    appState.simulation = normalizeSimulation(result);
    appState.viewport = computeViewport(appState.simulation.frames);
    renderStats(appState.simulation.stats);
    startAnimation();
    setStatus('Simulation complete.');
  } catch (error) {
    console.error(error);
    setStatus(error.message || 'Simulation failed.', true);
    renderStats(null);
    drawIdlePose();
  } finally {
    pyParams.destroy();
    pyResult?.destroy?.();
    hideLoading();
  }
}

function startAnimation() {
  if (!appState.simulation) return;
  appState.animationStart = null;

  const tick = (timestamp) => {
    if (!appState.animationStart) {
      appState.animationStart = timestamp;
    }
    const elapsed = (timestamp - appState.animationStart) / 1000;
    const index = Math.min(
      Math.floor(elapsed * appState.simulation.fps),
      appState.simulation.times.length - 1
    );
    drawFrame(index);

    if (index < appState.simulation.times.length - 1) {
      appState.animationHandle = requestAnimationFrame(tick);
    } else {
      appState.animationHandle = null;
    }
  };

  appState.animationHandle = requestAnimationFrame(tick);
}

async function initPyodideApp() {
  try {
    showLoading('Loading Pyodide runtime…');
    setStatus('Loading Pyodide and scientific packages…');
    appState.pyodide = await loadPyodide();
    await appState.pyodide.loadPackage(['numpy', 'scipy']);
    const physicsSource = await fetch('./trebuchet_physics.py').then((response) => response.text());
    await appState.pyodide.runPythonAsync(physicsSource);
    appState.simulateFn = appState.pyodide.globals.get('simulate');
    appState.ready = true;
    ui.fireButton.disabled = false;
    setStatus('Ready. Press Fire to launch.');
  } catch (error) {
    console.error(error);
    setStatus(`Pyodide failed to load: ${error.message}`, true);
  } finally {
    hideLoading();
  }
}

function resetSimulation() {
  clearSimulationState();
  setStatus(appState.ready ? 'Ready. Press Fire to launch.' : 'Still loading Pyodide…');
  drawIdlePose();
}

function sizeCanvas() {
  const rect = ui.canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  ui.canvas.width = Math.round(rect.width * dpr);
  ui.canvas.height = Math.round(rect.height * dpr);
  ui.ctx.setTransform(1, 0, 0, 1, 0, 0);
  ui.ctx.scale(dpr, dpr);
  ui.canvas.width = rect.width;
  ui.canvas.height = rect.height;
  drawIdlePose();
}

function init() {
  buildForm();
  renderStats(null);
  drawIdlePose();
  ui.fireButton.addEventListener('click', runSimulation);
  ui.resetButton.addEventListener('click', resetSimulation);
  window.addEventListener('resize', drawIdlePose);
  initPyodideApp();
}

init();
