import { TrebuchetRenderer } from './renderer';
import { findSampleAtTime, type SimulationResult, type TrebuchetParams } from './physics';
import { designs, defaultDesignId, type TrebuchetDesign, type ParameterConfig } from './designs';

// State
let renderer: TrebuchetRenderer;
let currentDesign: TrebuchetDesign = designs[defaultDesignId];
let currentResult: SimulationResult | null = null;
let animationFrame = 0;
let animationStart = 0;
let paused = true;
let playSpeed = 1;
let pausedTime = 0;

// Initialize
document.addEventListener('DOMContentLoaded', () => {
  const canvas = document.getElementById('simulationCanvas') as HTMLCanvasElement;
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width;
  canvas.height = rect.height;

  renderer = new TrebuchetRenderer(canvas);
  renderer.setGeometryFunction(currentDesign.computeGeometry);

  buildDesignSelector();
  buildParameterInputs();
  setupControls();
  renderPreview();

  document.getElementById('pauseBtn')!.textContent = 'Fire';
});

window.addEventListener('resize', () => {
  const canvas = document.getElementById('simulationCanvas') as HTMLCanvasElement;
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width;
  canvas.height = rect.height;

  if (currentResult) {
    renderer.setSimulation(currentResult);
    renderer.render(currentResult, pausedTime);
  } else {
    renderPreview();
  }
});

function buildDesignSelector(): void {
  const select = document.getElementById('trebuchetType') as HTMLSelectElement;
  if (!select) return;
  select.innerHTML = '';

  for (const design of Object.values(designs)) {
    const option = document.createElement('option');
    option.value = design.id;
    option.textContent = design.name;
    if (design.id === currentDesign.id) option.selected = true;
    select.appendChild(option);
  }

  select.addEventListener('change', () => {
    const design = designs[select.value];
    if (!design) return;
    switchDesign(design);
  });
}

function switchDesign(design: TrebuchetDesign): void {
  currentDesign = design;
  renderer.setGeometryFunction(design.computeGeometry);

  // Reset simulation state
  cancelAnimationFrame(animationFrame);
  currentResult = null;
  paused = true;
  pausedTime = 0;
  document.getElementById('pauseBtn')!.textContent = 'Fire';

  buildParameterInputs();
  clearStats();
  renderPreview();
}

function buildParameterInputs(): void {
  const container = document.querySelector('.section .parameters') as HTMLElement;
  if (!container) return;
  container.innerHTML = '';

  currentDesign.parameterConfig.forEach(param => {
    const group = document.createElement('div');
    group.className = 'param-group';

    const label = document.createElement('label');
    label.setAttribute('for', param.id);
    label.textContent = param.label;

    const input = document.createElement('input');
    input.type = 'number';
    input.id = param.id;
    input.step = String(param.step);
    input.min = String(param.min);
    input.max = String(param.max);
    input.value = String(param.default);

    const unit = document.createElement('span');
    unit.className = 'unit';
    unit.textContent = param.unit;

    group.appendChild(label);
    group.appendChild(input);
    group.appendChild(unit);
    container.appendChild(group);

    input.addEventListener('change', () => {
      if (!currentResult) {
        renderPreview();
      }
    });
  });
}

function readPhysicsParams(): Partial<TrebuchetParams> {
  const params: Partial<TrebuchetParams> = {};
  currentDesign.parameterConfig.forEach(config => {
    const input = document.getElementById(config.id) as HTMLInputElement;
    if (input && input.value) {
      (params as any)[config.physicsKey] = parseFloat(input.value);
    }
  });
  return params;
}

function renderPreview(): void {
  try {
    const { params, sample } = currentDesign.createInitialSample(readPhysicsParams());
    renderer.drawPreview(params, sample);
  } catch (e) {
    console.error('renderPreview failed:', e);
  }
}

function setupControls(): void {
  const pauseBtn = document.getElementById('pauseBtn')!;
  const resetBtn = document.getElementById('resetBtn')!;
  const stepBtn = document.getElementById('stepBtn')!;
  const stepMultiBtn = document.getElementById('stepMultiBtn')!;
  const playSpeedSlider = document.getElementById('playSpeed') as HTMLInputElement;
  const playSpeedValue = document.getElementById('playSpeedValue')!;

  pauseBtn.addEventListener('click', () => {
    if (!currentResult) {
      // Fire the trebuchet
      fire();
    } else if (paused) {
      // Resume animation
      paused = false;
      animationStart = performance.now() - (pausedTime * 1000 / playSpeed);
      pauseBtn.textContent = 'Pause';
      tick();
    } else {
      // Pause animation
      paused = true;
      cancelAnimationFrame(animationFrame);
      pauseBtn.textContent = 'Play';
    }
  });

  resetBtn.addEventListener('click', () => {
    cancelAnimationFrame(animationFrame);
    currentResult = null;
    paused = true;
    pausedTime = 0;
    pauseBtn.textContent = 'Fire';
    clearStats();
    renderPreview();
  });

  stepBtn.addEventListener('click', () => {
    if (!currentResult) fire();
    if (currentResult) {
      pausedTime = Math.min(pausedTime + 1 / 60, currentResult.stats.totalTime);
      paused = true;
      pauseBtn.textContent = 'Play';
      renderer.render(currentResult, pausedTime);
      updateStatsAtTime(pausedTime);
    }
  });

  stepMultiBtn.addEventListener('click', () => {
    const stepCount = parseInt((document.getElementById('stepCount') as HTMLInputElement).value) || 10;
    if (!currentResult) fire();
    if (currentResult) {
      pausedTime = Math.min(pausedTime + stepCount / 60, currentResult.stats.totalTime);
      paused = true;
      pauseBtn.textContent = 'Play';
      renderer.render(currentResult, pausedTime);
      updateStatsAtTime(pausedTime);
    }
  });

  if (playSpeedSlider) {
    playSpeedSlider.addEventListener('input', () => {
      playSpeed = parseFloat(playSpeedSlider.value);
      playSpeedValue.textContent = playSpeed + 'x';
      if (!paused && currentResult) {
        animationStart = performance.now() - (pausedTime * 1000 / playSpeed);
      }
    });
  }

  // Keyboard shortcuts
  document.addEventListener('keydown', (e) => {
    if (e.code === 'Space') {
      e.preventDefault();
      pauseBtn.click();
    } else if (e.code === 'Enter') {
      e.preventDefault();
      resetBtn.click();
    } else if (e.code === 'ArrowRight') {
      e.preventDefault();
      stepBtn.click();
    }
  });
}

function fire(): void {
  cancelAnimationFrame(animationFrame);

  try {
    const params = readPhysicsParams();
    currentResult = currentDesign.simulate(params);
    renderer.setSimulation(currentResult);
    updateFinalStats(currentResult);
    animationStart = performance.now();
    pausedTime = 0;
    paused = false;
    document.getElementById('pauseBtn')!.textContent = 'Pause';
    tick();
  } catch (error) {
    const msg = error instanceof Error ? error.message : 'Unknown error';
    console.error('Simulation error:', msg);
    currentResult = null;
    renderPreview();
  }
}

function tick(): void {
  if (!currentResult || paused) return;

  const elapsed = (performance.now() - animationStart) / 1000 * playSpeed;
  const displayTime = Math.min(elapsed, currentResult.stats.totalTime);
  pausedTime = displayTime;

  renderer.render(currentResult, displayTime);
  updateStatsAtTime(displayTime);

  if (displayTime < currentResult.stats.totalTime) {
    animationFrame = requestAnimationFrame(tick);
  } else {
    paused = true;
    document.getElementById('pauseBtn')!.textContent = 'Fire';
  }
}

function updateStatsAtTime(time: number): void {
  if (!currentResult) return;
  const sample = findSampleAtTime(currentResult.samples, time);
  const params = currentResult.params;

  setStatValue('distanceValue', Math.abs(sample.projectileX - currentResult.samples[0].projectileX).toFixed(1) + ' m');
  setStatValue('currentHeightValue', (params.h - sample.projectileY).toFixed(1) + ' m');
  setStatValue('velocityValue', sample.projectileSpeed.toFixed(1) + ' m/s');
  setStatValue('timeValue', time.toFixed(2) + ' s');
  setStatValue('maxDistanceValue', currentResult.stats.range.toFixed(1) + ' m');
  setStatValue('heightValue', currentResult.stats.maxHeight.toFixed(1) + ' m');
  setStatValue('maxVelocityValue', currentResult.stats.peakSpeed.toFixed(1) + ' m/s');
  setStatValue('estimatedDistanceValue', currentResult.stats.range.toFixed(1) + ' m');
}

function updateFinalStats(result: SimulationResult): void {
  setStatValue('maxDistanceValue', result.stats.range.toFixed(1) + ' m');
  setStatValue('heightValue', result.stats.maxHeight.toFixed(1) + ' m');
  setStatValue('maxVelocityValue', result.stats.peakSpeed.toFixed(1) + ' m/s');
  setStatValue('estimatedDistanceValue', result.stats.range.toFixed(1) + ' m');
}

function clearStats(): void {
  const ids = ['distanceValue', 'currentHeightValue', 'velocityValue', 'timeValue',
    'maxDistanceValue', 'heightValue', 'maxVelocityValue', 'estimatedDistanceValue'];
  ids.forEach(id => setStatValue(id, '0'));
}

function setStatValue(id: string, value: string): void {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}
