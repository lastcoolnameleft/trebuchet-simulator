import { TrebuchetRenderer } from './renderer';
import {
  createInitialSample,
  defaultParams,
  simulateTrebuchet,
  type SimulationResult,
  type TrebuchetParams,
} from './physics';

const fields: Array<{ key: keyof TrebuchetParams; label: string; step?: string }> = [
  { key: 'LAl', label: 'Arm long side [m]', step: '0.1' },
  { key: 'LAs', label: 'Arm short side [m]', step: '0.1' },
  { key: 'LAcg', label: 'Arm CG distance [m]', step: '0.1' },
  { key: 'LW', label: 'CW hanging length [m]', step: '0.1' },
  { key: 'LS', label: 'Sling length [m]', step: '0.1' },
  { key: 'h', label: 'Pivot height [m]', step: '0.1' },
  { key: 'mA', label: 'Arm mass [kg]', step: '1' },
  { key: 'mW', label: 'Counterweight mass [kg]', step: '1' },
  { key: 'mP', label: 'Projectile mass [kg]', step: '0.1' },
  { key: 'IA3', label: 'Arm inertia [kg·m²]', step: '0.1' },
  { key: 'IW3', label: 'CW inertia [kg·m²]', step: '0.1' },
  { key: 'releaseAngle', label: 'Release angle [deg]', step: '0.1' },
  { key: 'startArmAngleDeg', label: 'Start arm angle [deg]', step: '0.1' },
  { key: 'Grav', label: 'Gravity [m/s²]', step: '0.01' },
];

const form = document.getElementById('controls') as HTMLFormElement;
const fireButton = document.getElementById('fireButton') as HTMLButtonElement;
const resetButton = document.getElementById('resetButton') as HTMLButtonElement;
const statsRoot = document.getElementById('stats') as HTMLDivElement;
const status = document.getElementById('status') as HTMLDivElement;
const canvas = document.getElementById('simCanvas') as HTMLCanvasElement;

const renderer = new TrebuchetRenderer(canvas);
const defaults = defaultParams();
let currentResult: SimulationResult | null = null;
let animationFrame = 0;
let animationStart = 0;

buildForm(defaults);
renderPreview();
status.innerHTML = '<strong>Ready.</strong> Adjust parameters and press Fire.';

window.addEventListener('resize', () => {
  if (currentResult) {
    renderer.setSimulation(currentResult);
    renderer.render(currentResult, 0);
    return;
  }
  renderPreview();
});

form.addEventListener('input', () => {
  if (!currentResult) {
    renderPreview();
  }
});

fireButton.addEventListener('click', () => {
  cancelAnimationFrame(animationFrame);

  try {
    const params = readParams();
    currentResult = simulateTrebuchet(params);
    renderer.setSimulation(currentResult);
    updateStats(currentResult);
    animationStart = performance.now();
    status.innerHTML = '<strong>Simulating.</strong> Analytical RK4 trajectory is playing back at 1× speed.';
    tick();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Unknown simulation error.';
    status.innerHTML = `<strong>Simulation error.</strong> ${message}`;
    currentResult = null;
    renderPreview();
  }
});

resetButton.addEventListener('click', () => {
  cancelAnimationFrame(animationFrame);
  currentResult = null;
  updateStats(null);
  renderPreview();
  status.innerHTML = '<strong>Reset.</strong> Trebuchet returned to its pre-fire pose.';
});

function tick(): void {
  if (!currentResult) {
    return;
  }

  const elapsed = (performance.now() - animationStart) / 1000;
  const displayTime = Math.min(elapsed, currentResult.stats.totalTime);
  renderer.render(currentResult, displayTime);

  if (displayTime < currentResult.stats.totalTime) {
    animationFrame = requestAnimationFrame(tick);
    return;
  }

  status.innerHTML = `<strong>Impact.</strong> Range ${currentResult.stats.range.toFixed(2)} m after ${currentResult.stats.totalTime.toFixed(2)} s.`;
}

function renderPreview(): void {
  const { params, sample } = createInitialSample(readParamsSafe());
  renderer.drawPreview(params, sample);
}

function buildForm(params: TrebuchetParams): void {
  form.innerHTML = '';
  fields.forEach(({ key, label, step }) => {
    const wrapper = document.createElement('label');
    wrapper.innerHTML = `${label}<input name="${key}" type="number" step="${step ?? 'any'}" value="${params[key]}" />`;
    form.appendChild(wrapper);
  });
}

function readParams(): Partial<TrebuchetParams> {
  const values: Partial<TrebuchetParams> = {};
  new FormData(form).forEach((value, key) => {
    values[key as keyof TrebuchetParams] = Number(value) as never;
  });

  for (const [key, value] of Object.entries(values)) {
    if (!Number.isFinite(value)) {
      throw new Error(`Invalid numeric value for ${key}.`);
    }
  }

  return values;
}

function readParamsSafe(): Partial<TrebuchetParams> {
  try {
    return readParams();
  } catch {
    return defaults;
  }
}

function updateStats(result: SimulationResult | null): void {
  const cards = result
    ? [
        ['Range', `${result.stats.range.toFixed(2)} m`],
        ['Max height', `${result.stats.maxHeight.toFixed(2)} m`],
        ['Peak speed', `${result.stats.peakSpeed.toFixed(2)} m/s`],
        ['Release speed', `${result.stats.releaseSpeed.toFixed(2)} m/s`],
        ['Lift-off', `${result.stats.liftOffTime.toFixed(3)} s`],
        ['Flight time', `${result.stats.flightTime.toFixed(3)} s`],
        ['Release height', `${result.stats.releaseHeight.toFixed(2)} m`],
        ['Total time', `${result.stats.totalTime.toFixed(3)} s`],
      ]
    : [
        ['Range', '—'],
        ['Max height', '—'],
        ['Peak speed', '—'],
        ['Release speed', '—'],
        ['Lift-off', '—'],
        ['Flight time', '—'],
        ['Release height', '—'],
        ['Total time', '—'],
      ];

  statsRoot.innerHTML = cards
    .map(
      ([label, value]) =>
        `<div class="stat"><div class="label">${label}</div><div class="value">${value}</div></div>`,
    )
    .join('');
}

updateStats(null);
