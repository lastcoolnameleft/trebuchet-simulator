import {
  computeArmCgVelocity,
  computeCounterweightVelocity,
  computeProjectileAcceleration,
  computeProjectileVelocity,
  computeTrebuchetGeometry,
} from './geometry';

const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;
const FIXED_DT = 0.001;
const MAX_TIME = 12;
const EPSILON = 1e-6;

export interface TrebuchetParams {
  LAl: number;
  LAs: number;
  LAcg: number;
  LW: number;
  LS: number;
  h: number;
  mA: number;
  mW: number;
  mP: number;
  IA3: number;
  IW3: number;
  releaseAngle: number;
  Grav: number;
  startArmAngleDeg: number;
}

export interface SimulationSample {
  time: number;
  stage: 'ground' | 'lifted' | 'flight' | 'done';
  Aq: number;
  Wq: number;
  Sq: number;
  Aw: number;
  Ww: number;
  Sw: number;
  projectileX: number;
  projectileY: number;
  projectileVx: number;
  projectileVy: number;
  projectileSpeed: number;
  releaseAngleNow: number;
}

export interface SimulationStats {
  range: number;
  maxHeight: number;
  peakSpeed: number;
  releaseSpeed: number;
  releaseHeight: number;
  releaseTime: number;
  flightTime: number;
  totalTime: number;
  liftOffTime: number;
}

export interface SimulationResult {
  params: TrebuchetParams;
  samples: SimulationSample[];
  stats: SimulationStats;
}

export interface LiftedState {
  Aq: number;
  Wq: number;
  Sq: number;
  Aw: number;
  Ww: number;
  Sw: number;
}

interface FreeFlightState {
  x: number;
  y: number;
  vx: number;
  vy: number;
  Aq: number;
  Wq: number;
  Aw: number;
  Ww: number;
}

interface EnergySystem {
  kinetic: (q: number[], qd: number[]) => number;
  potential: (q: number[]) => number;
}

export const defaultParams = (): TrebuchetParams => {
  const LAl = 6;
  const LAs = 2;
  const mA = 50;
  const LW = 2;
  const mW = 200;

  return {
    LAl,
    LAs,
    LAcg: (LAs - LAl) / 2,
    LW,
    LS: 6,
    h: 5,
    mA,
    mW,
    mP: 10,
    IA3: (mA * (LAl + LAs) ** 2) / 12,
    IW3: (mW * LW ** 2) / 12,
    releaseAngle: 45,
    Grav: 9.81,
    startArmAngleDeg: 45,
  };
};

export function normalizeParams(overrides: Partial<TrebuchetParams> = {}): TrebuchetParams {
  const base = defaultParams();
  const merged = { ...base, ...overrides };

  return {
    ...merged,
    LAcg: overrides.LAcg ?? (merged.LAs - merged.LAl) / 2,
    IA3: overrides.IA3 ?? (merged.mA * (merged.LAl + merged.LAs) ** 2) / 12,
    IW3: overrides.IW3 ?? (merged.mW * merged.LW ** 2) / 12,
  };
}

export function createInitialSample(input: Partial<TrebuchetParams> = {}): { params: TrebuchetParams; sample: SimulationSample } {
  const params = normalizeParams(input);
  const Aq = params.startArmAngleDeg * DEG_TO_RAD;
  const Wq = 0;
  const Sq = solveGroundSlingAngle(params, Aq);
  const sample = makeSample(params, 0, 'ground', {
    Aq,
    Wq,
    Sq,
    Aw: 0,
    Ww: 0,
    Sw: 0,
  });
  return { params, sample };
}

export function simulateTrebuchet(input: Partial<TrebuchetParams> = {}): SimulationResult {
  const params = normalizeParams(input);
  const samples: SimulationSample[] = [];
  const initialAq = params.startArmAngleDeg * DEG_TO_RAD;
  let Aq = initialAq;
  let Wq = 0;
  let Sq = solveGroundSlingAngle(params, Aq);
  let Aw = 0;
  let Ww = 0;
  let Sw = groundSlingRate(params, Aq, Sq, Aw);
  let time = 0;
  let stage: SimulationSample['stage'] = 'ground';
  let liftOffTime = 0;
  let releaseTime = 0;
  let releasedSlingAngle = Sq;
  let releaseSpeed = 0;
  let releaseHeight = params.h - computeTrebuchetGeometry(params, { Aq, Wq, Sq }).projectile.y;
  let peakSpeed = 0;
  let maxHeight = 0;
  let lastReleaseDelta: number | null = null;
  let flight: FreeFlightState | null = null;

  const pushSample = (sample: SimulationSample) => {
    samples.push(sample);
    peakSpeed = Math.max(peakSpeed, sample.projectileSpeed);
    maxHeight = Math.max(maxHeight, params.h - sample.projectileY);
  };

  pushSample(makeSample(params, time, stage, { Aq, Wq, Sq, Aw, Ww, Sw }));

  while (time < MAX_TIME) {
    if (stage === 'ground') {
      const next = rk4([Aq, Wq, Aw, Ww], FIXED_DT, (state) => groundDerivative(params, state));
      Aq = next[0];
      Wq = next[1];
      Aw = next[2];
      Ww = next[3];
      Sq = solveGroundSlingAngle(params, Aq);
      Sw = groundSlingRate(params, Aq, Sq, Aw);
      time += FIXED_DT;

      const lifted = liftedAcceleration(params, { Aq, Wq, Sq, Aw, Ww, Sw });
      const projAcc = computeProjectileAcceleration(
        params,
        { Aq, Sq },
        { Aw, Sw },
        { Aacc: lifted.Aacc, Sacc: lifted.Sacc },
      );

      if (projAcc.y < 0) {
        stage = 'lifted';
        liftOffTime = time;
      }

      pushSample(makeSample(params, time, stage, { Aq, Wq, Sq, Aw, Ww, Sw }));
      continue;
    }

    if (stage === 'lifted') {
      const next = rk4([Aq, Wq, Sq, Aw, Ww, Sw], FIXED_DT, (state) => liftedDerivative(params, state));
      Aq = next[0];
      Wq = next[1];
      Sq = next[2];
      Aw = next[3];
      Ww = next[4];
      Sw = next[5];
      time += FIXED_DT;

      const currentSample = makeSample(params, time, stage, { Aq, Wq, Sq, Aw, Ww, Sw });
      const delta = currentSample.releaseAngleNow - params.releaseAngle;
      const readyToRelease = currentSample.projectileVx > 0;

      if (readyToRelease && lastReleaseDelta !== null && lastReleaseDelta > 0 && delta <= 0) {
        stage = 'flight';
        releaseTime = time;
        releaseSpeed = currentSample.projectileSpeed;
        releaseHeight = params.h - currentSample.projectileY;
        releasedSlingAngle = Sq;
        flight = {
          x: currentSample.projectileX,
          y: currentSample.projectileY,
          vx: currentSample.projectileVx,
          vy: currentSample.projectileVy,
          Aq,
          Wq,
          Aw,
          Ww,
        };
        pushSample({ ...currentSample, stage: 'flight' });
        continue;
      }

      lastReleaseDelta = readyToRelease ? delta : lastReleaseDelta;
      pushSample(currentSample);
      continue;
    }

    if (!flight) {
      break;
    }

    const next = rk4(
      [flight.x, flight.y, flight.vx, flight.vy, flight.Aq, flight.Wq, flight.Aw, flight.Ww],
      FIXED_DT,
      (state) => freeFlightDerivative(params, state),
    );

    flight = {
      x: next[0],
      y: next[1],
      vx: next[2],
      vy: next[3],
      Aq: next[4],
      Wq: next[5],
      Aw: next[6],
      Ww: next[7],
    };
    time += FIXED_DT;

    const sample = makeSample(
      params,
      time,
      'flight',
      {
        Aq: flight.Aq,
        Wq: flight.Wq,
        Sq: releasedSlingAngle,
        Aw: flight.Aw,
        Ww: flight.Ww,
        Sw: 0,
      },
      {
        x: flight.x,
        y: flight.y,
        vx: flight.vx,
        vy: flight.vy,
      },
    );

    pushSample(sample);

    if (flight.y > params.h) {
      stage = 'done';
      break;
    }
  }

  const lastSample = samples[samples.length - 1];
  const safeReleaseTime = releaseTime || lastSample.time;
  const launchReferenceX = samples[0]?.projectileX ?? 0;

  return {
    params,
    samples,
    stats: {
      range: Math.abs(lastSample.projectileX - launchReferenceX),
      maxHeight,
      peakSpeed,
      releaseSpeed,
      releaseHeight,
      releaseTime: safeReleaseTime,
      flightTime: Math.max(0, lastSample.time - safeReleaseTime),
      totalTime: lastSample.time,
      liftOffTime,
    },
  };
}

export function findSampleAtTime(samples: SimulationSample[], time: number): SimulationSample {
  if (samples.length === 0) {
    throw new Error('No simulation samples available.');
  }

  if (time <= samples[0].time) {
    return samples[0];
  }

  const last = samples[samples.length - 1];
  if (time >= last.time) {
    return last;
  }

  let low = 0;
  let high = samples.length - 1;

  while (low <= high) {
    const mid = Math.floor((low + high) / 2);
    if (samples[mid].time < time) {
      low = mid + 1;
    } else {
      high = mid - 1;
    }
  }

  const next = samples[low];
  const prev = samples[low - 1];
  const span = next.time - prev.time || FIXED_DT;
  const alpha = (time - prev.time) / span;

  return {
    time,
    stage: alpha < 0.5 ? prev.stage : next.stage,
    Aq: lerp(prev.Aq, next.Aq, alpha),
    Wq: lerp(prev.Wq, next.Wq, alpha),
    Sq: lerp(prev.Sq, next.Sq, alpha),
    Aw: lerp(prev.Aw, next.Aw, alpha),
    Ww: lerp(prev.Ww, next.Ww, alpha),
    Sw: lerp(prev.Sw, next.Sw, alpha),
    projectileX: lerp(prev.projectileX, next.projectileX, alpha),
    projectileY: lerp(prev.projectileY, next.projectileY, alpha),
    projectileVx: lerp(prev.projectileVx, next.projectileVx, alpha),
    projectileVy: lerp(prev.projectileVy, next.projectileVy, alpha),
    projectileSpeed: lerp(prev.projectileSpeed, next.projectileSpeed, alpha),
    releaseAngleNow: lerp(prev.releaseAngleNow, next.releaseAngleNow, alpha),
  };
}

function makeSample(
  params: TrebuchetParams,
  time: number,
  stage: SimulationSample['stage'],
  liftedState: LiftedState,
  projectileOverride?: { x: number; y: number; vx: number; vy: number },
): SimulationSample {
  const geometry = computeTrebuchetGeometry(params, liftedState);
  const projectileVelocity = projectileOverride
    ? { x: projectileOverride.vx, y: projectileOverride.vy }
    : computeProjectileVelocity(params, liftedState, liftedState);
  const projectile = projectileOverride
    ? { x: projectileOverride.x, y: projectileOverride.y }
    : geometry.projectile;

  return {
    time,
    stage,
    Aq: liftedState.Aq,
    Wq: liftedState.Wq,
    Sq: liftedState.Sq,
    Aw: liftedState.Aw,
    Ww: liftedState.Ww,
    Sw: liftedState.Sw,
    projectileX: projectile.x,
    projectileY: projectile.y,
    projectileVx: projectileVelocity.x,
    projectileVy: projectileVelocity.y,
    projectileSpeed: Math.hypot(projectileVelocity.x, projectileVelocity.y),
    releaseAngleNow: Math.atan2(-projectileVelocity.y, projectileVelocity.x) * RAD_TO_DEG,
  };
}

function solveGroundSlingAngle(params: TrebuchetParams, Aq: number): number {
  const cosTerm = clamp((params.h - params.LAl * Math.cos(Aq)) / params.LS, -1, 1);
  return Math.acos(cosTerm) - Aq;
}

function groundSlingRate(params: TrebuchetParams, Aq: number, Sq: number, Aw: number): number {
  const denominator = params.LS * Math.sin(Aq + Sq);
  if (Math.abs(denominator) < EPSILON) {
    return 0;
  }
  return -((params.LAl * Math.sin(Aq) + params.LS * Math.sin(Aq + Sq)) * Aw) / denominator;
}

function groundDerivative(params: TrebuchetParams, state: number[]): number[] {
  const [Aq, Wq, Aw, Ww] = state;
  const system = createGroundSystem(params);
  const [Aacc, Wacc] = reducedAcceleration([Aq, Wq], [Aw, Ww], system);
  return [Aw, Ww, Aacc, Wacc];
}

function createGroundSystem(params: TrebuchetParams): EnergySystem {
  return {
    kinetic: ([Aq, Wq], [Aw, Ww]) => {
      const Sq = solveGroundSlingAngle(params, Aq);
      const Sw = groundSlingRate(params, Aq, Sq, Aw);
      const pose = { Aq, Wq, Sq };
      const armVelocity = computeArmCgVelocity(params, pose, { Aw });
      const weightVelocity = computeCounterweightVelocity(params, pose, { Aw, Ww });
      const projectileVelocity = computeProjectileVelocity(params, pose, { Aw, Sw });

      return (
        0.5 * params.mA * (armVelocity.x ** 2 + armVelocity.y ** 2) +
        0.5 * params.IA3 * Aw ** 2 +
        0.5 * params.mW * (weightVelocity.x ** 2 + weightVelocity.y ** 2) +
        0.5 * params.IW3 * (Aw + Ww) ** 2 +
        0.5 * params.mP * (projectileVelocity.x ** 2 + projectileVelocity.y ** 2)
      );
    },
    potential: ([Aq, Wq]) => {
      const Sq = solveGroundSlingAngle(params, Aq);
      const geometry = computeTrebuchetGeometry(params, { Aq, Wq, Sq });

      return -params.Grav * (params.mA * geometry.armCg.y + params.mW * geometry.counterweight.y + params.mP * geometry.projectile.y);
    },
  };
}

function createArmOnlySystem(params: TrebuchetParams): EnergySystem {
  return {
    kinetic: ([Aq, Wq], [Aw, Ww]) => {
      const pose = { Aq, Wq, Sq: 0 };
      const armVelocity = computeArmCgVelocity(params, pose, { Aw });
      const weightVelocity = computeCounterweightVelocity(params, pose, { Aw, Ww });

      return (
        0.5 * params.mA * (armVelocity.x ** 2 + armVelocity.y ** 2) +
        0.5 * params.IA3 * Aw ** 2 +
        0.5 * params.mW * (weightVelocity.x ** 2 + weightVelocity.y ** 2) +
        0.5 * params.IW3 * (Aw + Ww) ** 2
      );
    },
    potential: ([Aq, Wq]) => {
      const geometry = computeTrebuchetGeometry(params, { Aq, Wq, Sq: 0 });
      return -params.Grav * (params.mA * geometry.armCg.y + params.mW * geometry.counterweight.y);
    },
  };
}

function reducedAcceleration(q: number[], qd: number[], system: EnergySystem): number[] {
  const dof = q.length;
  const mass = buildMassMatrix(dof, q, system.kinetic);
  const dMdq = q.map((_, axis) => differentiateMassMatrix(dof, q, axis, system.kinetic));
  const gradU = q.map((_, axis) => centralDifference((shifted) => system.potential(shifted), q, axis));
  const coriolisAndGravity = new Array(dof).fill(0);

  for (let i = 0; i < dof; i += 1) {
    let total = gradU[i];
    for (let j = 0; j < dof; j += 1) {
      for (let k = 0; k < dof; k += 1) {
        const christoffel = 0.5 * (dMdq[k][i][j] + dMdq[j][i][k] - dMdq[i][j][k]);
        total += christoffel * qd[j] * qd[k];
      }
    }
    coriolisAndGravity[i] = total;
  }

  const rhs = coriolisAndGravity.map((value) => -value);
  return solveLinearSystem(mass, rhs);
}

function buildMassMatrix(dof: number, q: number[], kinetic: EnergySystem['kinetic']): number[][] {
  const matrix = Array.from({ length: dof }, () => new Array(dof).fill(0));
  const basis = Array.from({ length: dof }, (_, index) => unitVector(dof, index));
  const single = basis.map((vector) => kinetic(q, vector));

  for (let i = 0; i < dof; i += 1) {
    matrix[i][i] = 2 * single[i];
    for (let j = i + 1; j < dof; j += 1) {
      const sumVelocity = basis[i].map((value, index) => value + basis[j][index]);
      const offDiagonal = kinetic(q, sumVelocity) - single[i] - single[j];
      matrix[i][j] = offDiagonal;
      matrix[j][i] = offDiagonal;
    }
  }

  return matrix;
}

function differentiateMassMatrix(dof: number, q: number[], axis: number, kinetic: EnergySystem['kinetic']): number[][] {
  const plus = q.slice();
  const minus = q.slice();
  plus[axis] += EPSILON;
  minus[axis] -= EPSILON;

  const mPlus = buildMassMatrix(dof, plus, kinetic);
  const mMinus = buildMassMatrix(dof, minus, kinetic);
  return mPlus.map((row, rowIndex) => row.map((value, colIndex) => (value - mMinus[rowIndex][colIndex]) / (2 * EPSILON)));
}

function liftedDerivative(params: TrebuchetParams, state: number[]): number[] {
  const [Aq, Wq, Sq, Aw, Ww, Sw] = state;
  const accel = liftedAcceleration(params, { Aq, Wq, Sq, Aw, Ww, Sw });
  return [Aw, Ww, Sw, accel.Aacc, accel.Wacc, accel.Sacc];
}

function liftedAcceleration(params: TrebuchetParams, state: LiftedState): { Aacc: number; Wacc: number; Sacc: number } {
  const { Aq, Wq, Sq, Aw, Ww, Sw } = state;
  const {
    IA3,
    IW3,
    Grav,
    LAcg,
    LAl,
    LAs,
    LS,
    LW,
    mA,
    mP,
    mW,
  } = params;

  const M11 =
    IA3 +
    IW3 +
    mA * LAcg ** 2 +
    mP * (LAl ** 2 + LS ** 2 + 2 * LAl * LS * Math.cos(Sq)) +
    mW * (LAs ** 2 + LW ** 2 + 2 * LAs * LW * Math.cos(Wq));
  const M12 = IW3 + LW * mW * (LW + LAs * Math.cos(Wq));
  const M13 = LS * mP * (LS + LAl * Math.cos(Sq));
  const M22 = IW3 + mW * LW ** 2;
  const M33 = mP * LS ** 2;

  const r1 =
    Grav * LAcg * mA * Math.sin(Aq) +
    Grav * mP * (LAl * Math.sin(Aq) + LS * Math.sin(Aq + Sq)) -
    Grav * mW * (LAs * Math.sin(Aq) + LW * Math.sin(Aq + Wq)) -
    LAl * LS * mP * Math.sin(Sq) * (Aw ** 2 - (Aw + Sw) ** 2) -
    LAs * LW * mW * Math.sin(Wq) * (Aw ** 2 - (Aw + Ww) ** 2);
  const r2 = -LW * mW * (Grav * Math.sin(Aq + Wq) + LAs * Math.sin(Wq) * Aw ** 2);
  const r3 = LS * mP * (Grav * Math.sin(Aq + Sq) - LAl * Math.sin(Sq) * Aw ** 2);

  const [Aacc, Wacc, Sacc] = solveLinearSystem(
    [
      [M11, M12, M13],
      [M12, M22, 0],
      [M13, 0, M33],
    ],
    [r1, r2, r3],
  );

  return { Aacc, Wacc, Sacc };
}

function freeFlightDerivative(params: TrebuchetParams, state: number[]): number[] {
  const [x, y, vx, vy, Aq, Wq, Aw, Ww] = state;
  const [Aacc, Wacc] = reducedAcceleration([Aq, Wq], [Aw, Ww], createArmOnlySystem(params));
  return [vx, vy, 0, params.Grav, Aw, Ww, Aacc, Wacc].map((value, index) => (index < 2 ? value : value));
}

function rk4(state: number[], dt: number, derivative: (state: number[]) => number[]): number[] {
  const k1 = derivative(state);
  const k2 = derivative(addScaled(state, k1, dt / 2));
  const k3 = derivative(addScaled(state, k2, dt / 2));
  const k4 = derivative(addScaled(state, k3, dt));

  return state.map((value, index) => value + (dt / 6) * (k1[index] + 2 * k2[index] + 2 * k3[index] + k4[index]));
}

function addScaled(state: number[], derivative: number[], scale: number): number[] {
  return state.map((value, index) => value + derivative[index] * scale);
}

function centralDifference(fn: (q: number[]) => number, q: number[], axis: number): number {
  const plus = q.slice();
  const minus = q.slice();
  plus[axis] += EPSILON;
  minus[axis] -= EPSILON;
  return (fn(plus) - fn(minus)) / (2 * EPSILON);
}

function solveLinearSystem(matrix: number[][], rhs: number[]): number[] {
  const size = rhs.length;
  const a = matrix.map((row) => row.slice());
  const b = rhs.slice();

  for (let pivot = 0; pivot < size; pivot += 1) {
    let maxRow = pivot;
    for (let row = pivot + 1; row < size; row += 1) {
      if (Math.abs(a[row][pivot]) > Math.abs(a[maxRow][pivot])) {
        maxRow = row;
      }
    }

    if (maxRow !== pivot) {
      [a[pivot], a[maxRow]] = [a[maxRow], a[pivot]];
      [b[pivot], b[maxRow]] = [b[maxRow], b[pivot]];
    }

    const pivotValue = a[pivot][pivot];
    if (Math.abs(pivotValue) < EPSILON) {
      throw new Error('Singular mass matrix encountered.');
    }

    for (let col = pivot; col < size; col += 1) {
      a[pivot][col] /= pivotValue;
    }
    b[pivot] /= pivotValue;

    for (let row = 0; row < size; row += 1) {
      if (row === pivot) {
        continue;
      }
      const factor = a[row][pivot];
      for (let col = pivot; col < size; col += 1) {
        a[row][col] -= factor * a[pivot][col];
      }
      b[row] -= factor * b[pivot];
    }
  }

  return b;
}

function unitVector(length: number, activeIndex: number): number[] {
  return Array.from({ length }, (_, index) => (index === activeIndex ? 1 : 0));
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function lerp(a: number, b: number, alpha: number): number {
  return a + (b - a) * alpha;
}
