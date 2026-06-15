const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;
const FIXED_DT = 0.001;
const MAX_TIME = 6.0;
const MAX_FLIGHT_TIME = 20.0;
const EPS = 1e-9;
const EVENT_EPS = 1e-7;
const AIR_DENSITY = 1.225;
const DRAG_COEFFICIENT = 0.47;
const PROJECTILE_DIAMETER = 0.0759; // Baseball diameter in meters (0.249 ft)
const WIND_SPEED = 0.0;
const ENABLE_AIR_DRAG = true;

type StageName = 'ground' | 'lifted' | 'flight' | 'done';
type StageState = [number, number, number, number, number, number];
type PostReleaseState = [number, number, number, number];
type FlightState = [number, number, number, number];

type InternalParams = TrebuchetParams & {
  releaseAngleRad: number;
  maxTime: number;
  maxFlightTime: number;
  maxStep: number;
  enableAirDrag: boolean;
  airDensity: number;
  dragCoefficient: number;
  projectileDiameter: number;
  projectileArea: number;
  windSpeed: number;
};

interface TimedState<T extends number[]> {
  time: number;
  state: T;
}

interface StageBundle {
  stage1: Array<TimedState<StageState>>;
  stage2: Array<TimedState<StageState>>;
  postRelease: Array<TimedState<PostReleaseState>>;
  flight: Array<TimedState<FlightState>>;
  tLiftoff: number;
  tRelease: number;
  tLand: number;
  releaseState: StageState;
  releasePos: { x: number; y: number };
  releaseVel: { vx: number; vy: number };
  initialProjectileX: number;
}

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
  stage: StageName;
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
  releaseAngle?: number;
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

export const defaultParams = (): TrebuchetParams => ({
  LAl: 2.0726,
  LAs: 0.5334,
  LAcg: 0.7681,
  LW: 0.6096,
  LS: 2.0828,
  h: 1.524,
  mA: 4.8307,
  mW: 44.4933,
  mP: 0.1497,
  IA3: 2.7287,
  IW3: 0.04244,
  releaseAngle: 45,
  Grav: 9.81,
  startArmAngleDeg: 0, // 0 = auto-compute from geometry
});

export function normalizeParams(input: Partial<TrebuchetParams> = {}): TrebuchetParams {
  const base = defaultParams();
  const merged = { ...base, ...input };
  const projectileArea = Math.PI * (Math.max(PROJECTILE_DIAMETER, 0) / 2) ** 2;

  return {
    ...merged,
    LAcg: input.LAcg ?? merged.LAcg,
    IA3: input.IA3 ?? merged.IA3,
    IW3: input.IW3 ?? merged.IW3,
    releaseAngleRad: merged.releaseAngle * DEG_TO_RAD,
    maxTime: MAX_TIME,
    maxFlightTime: MAX_FLIGHT_TIME,
    maxStep: FIXED_DT,
    enableAirDrag: ENABLE_AIR_DRAG,
    airDensity: AIR_DENSITY,
    dragCoefficient: DRAG_COEFFICIENT,
    projectileDiameter: PROJECTILE_DIAMETER,
    projectileArea,
    windSpeed: WIND_SPEED,
  } as InternalParams;
}

export function createInitialSample(input: Partial<TrebuchetParams> = {}): { params: TrebuchetParams; sample: SimulationSample } {
  const params = normalizeParams(input) as InternalParams;
  validateGeometry(params);
  const Aq = computeInitialArmAngle(params);
  const Wq = -Aq; // CW hanging straight down
  const Sq = groundSlingAngle(Aq, params);
  const sample = makeAttachedSample(params, 0, 'ground', [Aq, Wq, Sq, 0, 0, 0]);
  return { params, sample };
}

export function simulateTrebuchet(input: Partial<TrebuchetParams> = {}): SimulationResult {
  const params = normalizeParams(input) as InternalParams;
  const bundle = solveBundle(params);
  const samples = buildSamples(bundle, params);

  let peakSpeed = 0;
  let maxHeight = 0;
  for (const sample of samples) {
    peakSpeed = Math.max(peakSpeed, sample.projectileSpeed);
    maxHeight = Math.max(maxHeight, params.h - sample.projectileY);
  }

  const releaseSpeed = Math.hypot(bundle.releaseVel.vx, bundle.releaseVel.vy);
  const releaseAngle = Math.atan2(bundle.releaseVel.vy, bundle.releaseVel.vx) * RAD_TO_DEG;
  const releaseHeight = bundle.releasePos.y;

  return {
    params,
    samples,
    stats: {
      range: bundle.flight[bundle.flight.length - 1].state[0] - bundle.initialProjectileX,
      maxHeight,
      peakSpeed,
      releaseSpeed,
      releaseAngle,
      releaseHeight,
      releaseTime: bundle.tRelease,
      flightTime: bundle.tLand - bundle.tRelease,
      totalTime: bundle.tLand,
      liftOffTime: bundle.tLiftoff,
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

function solveBundle(params: InternalParams): StageBundle {
  validateGeometry(params);

  const Aq0 = computeInitialArmAngle(params);
  const Wq0 = -Aq0; // CW hanging straight down
  const Sq0 = groundSlingAngle(Aq0, params);
  const y0Stage1: StageState = [Aq0, Wq0, Sq0, 0.0, 0.0, 0.0];
  const initialMetric = liftOffMetric(y0Stage1, params);

  let stage1: Array<TimedState<StageState>> = [{ time: 0, state: y0Stage1.slice() as StageState }];
  let tLiftoff = 0;
  let liftoffState = y0Stage1.slice() as StageState;

  if (initialMetric < 0) {
    const stage1Run = integrateUntilEvent(
      y0Stage1,
      0,
      params.maxTime,
      params.maxStep,
      (state) => stage1Ode(state, params),
      (state) => liftOffEventMetric(state, params),
      1,
    );

    if (!stage1Run.event) {
      throw new Error('Projectile never lifted off the ground within the simulation time.');
    }

    stage1 = stage1Run.trajectory;
    tLiftoff = stage1Run.event.time;
    const [Aq, Wq, _Sq, Aw, Ww] = stage1Run.event.state;
    const Sq = groundSlingAngle(Aq, params);
    const Sw = stage1SqDot(Aq, Sq, Aw, params);
    liftoffState = [Aq, Wq, Sq, Aw, Ww, Sw];
    replaceLast(stage1, { time: tLiftoff, state: liftoffState.slice() as StageState });
  }

  const stage2Run = integrateUntilEvent(
    liftoffState,
    tLiftoff,
    params.maxTime,
    params.maxStep,
    (state) => stage2Ode(state, params),
    (state) => releaseEvent(state, params),
    -1,
  );

  if (!stage2Run.event) {
    throw new Error('Release angle was never reached. Try a lower release angle or different geometry.');
  }

  const stage2 = stage2Run.trajectory;
  const tRelease = stage2Run.event.time;
  const releaseState = stage2Run.event.state.slice() as StageState;
  replaceLast(stage2, { time: tRelease, state: releaseState.slice() as StageState });

  const releaseKinematics = projectileKinematics(releaseState, params);
  const releasePos = { x: releaseKinematics.x, y: releaseKinematics.y };
  const releaseVel = { vx: releaseKinematics.vx, vy: releaseKinematics.vy };

  const y0Post: PostReleaseState = [releaseState[0], releaseState[1], releaseState[3], releaseState[4]];
  const y0Flight: FlightState = [releaseKinematics.x, releaseKinematics.y, releaseKinematics.vx, releaseKinematics.vy];
  const tFlightEnd = tRelease + params.maxFlightTime;

  const postRelease: Array<TimedState<PostReleaseState>> = [{ time: tRelease, state: y0Post.slice() as PostReleaseState }];
  const flight: Array<TimedState<FlightState>> = [{ time: tRelease, state: y0Flight.slice() as FlightState }];

  let currentPost = y0Post.slice() as PostReleaseState;
  let currentFlight = y0Flight.slice() as FlightState;
  let currentTime = tRelease;
  let tLand = tRelease;
  let landed = false;

  while (currentTime < tFlightEnd - EPS) {
    const dt = Math.min(params.maxStep, tFlightEnd - currentTime);
    const nextPost = rk4Step(currentPost, dt, (state) => postReleaseOde(state, params));
    const nextFlight = rk4Step(currentFlight, dt, (state) => flightOde(state, params));
    const nextTime = currentTime + dt;
    const yPrev = currentFlight[1];
    const yNext = nextFlight[1];

    if (yPrev > 0 && yNext <= 0) {
      const refined = refineEvent(
        currentTime,
        currentFlight,
        nextTime,
        nextFlight,
        (state) => state[1],
        (state, delta) => rk4Step(state, delta, (inner) => flightOde(inner, params)),
        -1,
      );
      const postAtLand = rk4Step(currentPost, refined.time - currentTime, (state) => postReleaseOde(state, params));
      tLand = refined.time;
      currentPost = postAtLand;
      currentFlight = refined.state;
      postRelease.push({ time: tLand, state: postAtLand });
      flight.push({ time: tLand, state: refined.state });
      landed = true;
      break;
    }

    currentTime = nextTime;
    currentPost = nextPost;
    currentFlight = nextFlight;
    postRelease.push({ time: currentTime, state: currentPost });
    flight.push({ time: currentTime, state: currentFlight });
  }

  if (!landed) {
    throw new Error('Projectile did not land within the flight time limit.');
  }

  const initialProjectileX = projectileKinematics(y0Stage1, params).x;

  return {
    stage1,
    stage2,
    postRelease,
    flight,
    tLiftoff,
    tRelease,
    tLand,
    releaseState,
    releasePos,
    releaseVel,
    initialProjectileX,
  };
}

function buildSamples(bundle: StageBundle, params: InternalParams): SimulationSample[] {
  const samples: SimulationSample[] = [];

  for (const point of bundle.stage1) {
    pushSample(samples, makeAttachedSample(params, point.time, 'ground', point.state));
  }

  for (const point of bundle.stage2) {
    pushSample(samples, makeAttachedSample(params, point.time, 'lifted', point.state));
  }

  const releaseSq = bundle.releaseState[2];
  const releaseMech = bundle.postRelease[0].state; // freeze arm at release position
  for (let index = 0; index < bundle.flight.length; index += 1) {
    const point = bundle.flight[index];
    pushSample(samples, makeFlightSample(params, point.time, releaseMech, releaseSq, point.state));
  }

  return samples;
}

function pushSample(samples: SimulationSample[], sample: SimulationSample): void {
  const last = samples[samples.length - 1];
  if (last && Math.abs(last.time - sample.time) < EVENT_EPS) {
    samples[samples.length - 1] = sample;
    return;
  }
  samples.push(sample);
}

function makeAttachedSample(params: InternalParams, time: number, stage: StageName, state: StageState): SimulationSample {
  let normalizedState: StageState;
  if (stage === 'ground') {
    const Sq = groundSlingAngle(state[0], params);
    normalizedState = [state[0], state[1], Sq, state[3], state[4], stage1SqDot(state[0], Sq, state[3], params)];
  } else {
    normalizedState = state.slice() as StageState;
  }
  const { x, y, vx, vy } = projectileKinematics(normalizedState, params);
  // Convert from physics (y=height above ground, vy=up) to renderer (y=below pivot, vy=down)
  return makeSample(time, stage, normalizedState, x, params.h - y, vx, -vy);
}

function makeFlightSample(
  params: InternalParams,
  time: number,
  mech: PostReleaseState,
  releaseSq: number,
  flightState: FlightState,
): SimulationSample {
  const state: StageState = [mech[0], mech[1], releaseSq, mech[2], mech[3], 0];
  // Flight state y = height above ground, vy = up. Convert for renderer.
  return makeSample(time, 'flight', state, flightState[0], params.h - flightState[1], flightState[2], -flightState[3]);
}

function makeSample(
  time: number,
  stage: StageName,
  state: StageState,
  projectileX: number,
  projectileY: number,
  projectileVx: number,
  projectileVy: number,
): SimulationSample {
  return {
    time,
    stage,
    Aq: state[0],
    Wq: state[1],
    Sq: state[2],
    Aw: state[3],
    Ww: state[4],
    Sw: state[5],
    projectileX,
    projectileY,
    projectileVx,
    projectileVy,
    projectileSpeed: Math.hypot(projectileVx, projectileVy),
    releaseAngleNow: Math.atan2(-projectileVy, projectileVx) * RAD_TO_DEG,
  };
}

function computeInitialArmAngle(params: InternalParams): number {
  // VT formula: Aq0 = π - acos(h/LAl), arm tilted so tip is near ground level
  // If startArmAngleDeg is explicitly set (non-zero), use it
  if (params.startArmAngleDeg !== 0) {
    return params.startArmAngleDeg * DEG_TO_RAD;
  }
  const ratio = params.h / params.LAl;
  if (Math.abs(ratio) <= 1.0) {
    return Math.PI - Math.acos(ratio);
  }
  // h > LAl: arm can't reach ground even vertical, use maximum tilt
  return (14 * Math.PI) / 15;
}

function validateGeometry(params: InternalParams): void {
  const Aq0 = computeInitialArmAngle(params);
  // Ground constraint: cos(Aq+Sq) = (-h - LAl*cos(Aq)) / LS must be in [-1, 1]
  const reach = Math.abs((-params.h - params.LAl * Math.cos(Aq0)) / params.LS);
  if (reach > 1) {
    throw new Error('Initial geometry cannot place the projectile on the ground. Increase sling length, lower the pivot, or reduce the initial arm angle.');
  }
}

function groundSlingAngle(Aq: number, params: InternalParams): number {
  // Ground constraint (VT convention): LAl*cos(Aq) + LS*cos(Aq+Sq) = -h
  // Choose the solution where Aq+Sq is in (π, 2π) so sling points toward target
  const cosTotal = clamp((-params.h - params.LAl * Math.cos(Aq)) / params.LS, -1, 1);
  const total = 2.0 * Math.PI - Math.acos(cosTotal);
  return total - Aq;
}

function stage1SqDot(Aq: number, Sq: number, Aw: number, params: InternalParams): number {
  const sAs = Math.sin(Aq + Sq);
  return -safeDiv((params.LAl * Math.sin(Aq) + params.LS * sAs) * Aw, params.LS * sAs);
}

function stage1Components(Aq: number, Wq: number, Aw: number, Ww: number, params: InternalParams): {
  Sq: number;
  Sw: number;
  Awd: number;
  Wwd: number;
  Swd: number;
} {
  const { LAl, LAs, LAcg, LW, LS, Grav, mA, mW, mP, IA3, IW3 } = params;
  const Sq = groundSlingAngle(Aq, params);
  const sA = Math.sin(Aq);
  const cA = Math.cos(Aq);
  const cW = Math.cos(Wq);
  const sW = Math.sin(Wq);
  const sSq = Math.sin(Sq);
  const cSq = Math.cos(Sq);
  const sAs = Math.sin(Aq + Sq);
  const cAs = Math.cos(Aq + Sq);
  const Sw = stage1SqDot(Aq, Sq, Aw, params);

  const coupling = safeDiv(cAs * Sw * (Sw + 2 * Aw), sAs);
  const slope = safeDiv(cAs, sAs) + safeDiv(LAl * cA, LS * sAs);
  const accelTerm = coupling + slope * Aw ** 2;

  const M11 =
    -mP * LAl ** 2 * (-1 + safeDiv(2 * sA * cSq, sAs)) +
    IA3 +
    IW3 +
    mA * LAcg ** 2 +
    mP * LAl ** 2 * safeDiv(sA ** 2, sAs ** 2) +
    mW * (LAs ** 2 + LW ** 2 + 2 * LAs * LW * cW);
  const M12 = IW3 + LW * mW * (LW + LAs * cW);
  const M22 = IW3 + mW * LW ** 2;

  const r1 =
    Grav * LAcg * mA * sA +
    LAl * LS * mP * (sSq * (Aw + Sw) ** 2 + cSq * accelTerm) +
    LAl * mP * sA * safeDiv(LAl * sSq * Aw ** 2 - LS * accelTerm, sAs) -
    Grav * mW * (LAs * sA + LW * Math.sin(Aq + Wq)) -
    LAs * LW * mW * sW * (Aw ** 2 - (Aw + Ww) ** 2);
  const r2 = -LW * mW * (Grav * Math.sin(Aq + Wq) + LAs * sW * Aw ** 2);

  const det = M11 * M22 - M12 * M12;
  if (Math.abs(det) < EPS) {
    throw new Error('Stage 1 matrix became singular.');
  }

  const Awd = (r1 * M22 - r2 * M12) / det;
  const Wwd = -(r1 * M12 - r2 * M11) / det;
  const Swd =
    -safeDiv(cAs * Sw * (Sw + 2 * Aw), sAs) -
    (safeDiv(cAs, sAs) + safeDiv(LAl * cA, LS * sAs)) * Aw ** 2 -
    safeDiv((LAl * sA + LS * sAs) * Awd, LS * sAs);

  return { Sq, Sw, Awd, Wwd, Swd };
}

function stage1Ode(state: StageState, params: InternalParams): StageState {
  const [Aq, Wq, _Sq, Aw, Ww] = state;
  const { Sq, Sw, Awd, Wwd, Swd } = stage1Components(Aq, Wq, Aw, Ww, params);
  return [Aw, Ww, Sw, Awd, Wwd, Swd];
}

function stage2Ode(state: StageState, params: InternalParams): StageState {
  const [Aq, Wq, Sq, Aw, Ww, Sw] = state;
  const { LAl, LAs, LAcg, LW, LS, Grav, mA, mW, mP, IA3, IW3 } = params;
  const cSq = Math.cos(Sq);
  const sSq = Math.sin(Sq);
  const cW = Math.cos(Wq);
  const sW = Math.sin(Wq);

  const M11 = IA3 + IW3 + mA * LAcg ** 2 + mP * (LAl ** 2 + LS ** 2 + 2 * LAl * LS * cSq) + mW * (LAs ** 2 + LW ** 2 + 2 * LAs * LW * cW);
  const M12 = IW3 + LW * mW * (LW + LAs * cW);
  const M13 = LS * mP * (LS + LAl * cSq);
  const M22 = IW3 + mW * LW ** 2;
  const M33 = mP * LS ** 2;

  const r1 =
    Grav * LAcg * mA * Math.sin(Aq) +
    Grav * mP * (LAl * Math.sin(Aq) + LS * Math.sin(Aq + Sq)) -
    Grav * mW * (LAs * Math.sin(Aq) + LW * Math.sin(Aq + Wq)) -
    LAl * LS * mP * sSq * (Aw ** 2 - (Aw + Sw) ** 2) -
    LAs * LW * mW * sW * (Aw ** 2 - (Aw + Ww) ** 2);
  const r2 = -LW * mW * (Grav * Math.sin(Aq + Wq) + LAs * sW * Aw ** 2);
  const r3 = LS * mP * (Grav * Math.sin(Aq + Sq) - LAl * sSq * Aw ** 2);

  const [Awd, Wwd, Swd] = solve3x3(
    [
      [M11, M12, M13],
      [M12, M22, 0],
      [M13, 0, M33],
    ],
    [r1, r2, r3],
    'Stage 2 matrix became singular.',
  );

  return [Aw, Ww, Sw, Awd, Wwd, Swd];
}

function postReleaseOde(state: PostReleaseState, params: InternalParams): PostReleaseState {
  const [Aq, Wq, Aw, Ww] = state;
  const { LAs, LAcg, LW, Grav, mA, mW, IA3, IW3 } = params;
  const cW = Math.cos(Wq);
  const sW = Math.sin(Wq);

  const M11 = IA3 + IW3 + mA * LAcg ** 2 + mW * (LAs ** 2 + LW ** 2 + 2 * LAs * LW * cW);
  const M12 = IW3 + LW * mW * (LW + LAs * cW);
  const M22 = IW3 + mW * LW ** 2;
  const r1 = Grav * LAcg * mA * Math.sin(Aq) - Grav * mW * (LAs * Math.sin(Aq) + LW * Math.sin(Aq + Wq)) - LAs * LW * mW * sW * (Aw ** 2 - (Aw + Ww) ** 2);
  const r2 = -LW * mW * (Grav * Math.sin(Aq + Wq) + LAs * sW * Aw ** 2);

  const det = M11 * M22 - M12 * M12;
  if (Math.abs(det) < EPS) {
    throw new Error('Post-release matrix became singular.');
  }

  const Awd = (r1 * M22 - r2 * M12) / det;
  const Wwd = -(r1 * M12 - r2 * M11) / det;
  return [Aw, Ww, Awd, Wwd];
}

function projectileKinematics(state: StageState, params: InternalParams): { x: number; y: number; vx: number; vy: number } {
  const [Aq, _Wq, Sq, Aw, _Ww, Sw] = state;
  const x = -params.LAl * Math.sin(Aq) - params.LS * Math.sin(Aq + Sq);
  const y = params.h + params.LAl * Math.cos(Aq) + params.LS * Math.cos(Aq + Sq);
  const vx = -params.LAl * Math.cos(Aq) * Aw - params.LS * Math.cos(Aq + Sq) * (Aw + Sw);
  const vy = -params.LAl * Math.sin(Aq) * Aw - params.LS * Math.sin(Aq + Sq) * (Aw + Sw);
  return { x, y, vx, vy };
}

function stage1GroundReaction(state: StageState, params: InternalParams): number {
  const [Aq, Wq, _Sq, Aw, Ww] = state;
  const { Sq, Awd, Swd, Sw } = stage1Components(Aq, Wq, Aw, Ww, params);
  const total = Aq + Sq;
  const sTotal = Math.sin(total);
  const cTotal = Math.cos(total);

  const xdd =
    params.LAl * Math.sin(Aq) * Aw ** 2 -
    params.LAl * Math.cos(Aq) * Awd +
    params.LS * Math.sin(total) * (Aw + Sw) ** 2 -
    params.LS * Math.cos(total) * (Awd + Swd);

  if (Math.abs(sTotal) < 1e-6) {
    return 1.0;
  }

  const tension = (params.mP * xdd) / sTotal;
  return params.mP * params.Grav - tension * cTotal;
}

function liftOffMetric(state: StageState, params: InternalParams): number {
  const deriv = stage2Ode(state, params);
  const Awd = deriv[3];
  const Swd = deriv[5];
  const [Aq, _Wq, Sq, Aw, _Ww, Sw] = state;
  const total = Aq + Sq;

  return (
    params.LAl * Math.cos(Aq) * Aw ** 2 +
    params.LAl * Math.sin(Aq) * Awd +
    params.LS * Math.cos(total) * (Aw + Sw) ** 2 +
    params.LS * Math.sin(total) * (Awd + Swd)
  );
}

function liftOffEventMetric(state: StageState, params: InternalParams): number {
  const [Aq, Wq, _Sq, Aw, Ww] = state;
  const Sq = groundSlingAngle(Aq, params);
  const Sw = stage1SqDot(Aq, Sq, Aw, params);
  return liftOffMetric([Aq, Wq, Sq, Aw, Ww, Sw], params);
}

function releaseEvent(state: StageState, params: InternalParams): number {
  const { vx, vy } = projectileKinematics(state, params);
  const speed = Math.hypot(vx, vy);
  if (vx <= 0 || speed < 0.5) {
    return -1;
  }
  return Math.atan2(vy, vx) - params.releaseAngleRad;
}

function flightOde(state: FlightState, params: InternalParams): FlightState {
  const [x, y, vx, vy] = state;
  let dragTerm = 0;

  if (params.enableAirDrag) {
    const relX = vx - params.windSpeed;
    const speed = Math.hypot(relX, vy);
    dragTerm = (params.airDensity * params.dragCoefficient * params.projectileArea * speed) / (2 * params.mP);
  }

  const ax = -dragTerm * (vx - params.windSpeed);
  const ay = -params.Grav - dragTerm * vy;
  return [vx, vy, ax, ay];
}

function integrateUntilEvent<T extends number[]>(
  initialState: T,
  startTime: number,
  endTime: number,
  dt: number,
  derivative: (state: T) => T,
  metric: (state: T) => number,
  direction: 1 | -1,
): { trajectory: Array<TimedState<T>>; event: TimedState<T> | null } {
  const trajectory: Array<TimedState<T>> = [{ time: startTime, state: initialState.slice() as T }];
  let currentState = initialState.slice() as T;
  let currentTime = startTime;
  let currentMetric = metric(currentState);

  while (currentTime < endTime - EPS) {
    const step = Math.min(dt, endTime - currentTime);
    const nextState = rk4Step(currentState, step, derivative);
    const nextTime = currentTime + step;
    const nextMetric = metric(nextState);

    if (crossesEvent(currentMetric, nextMetric, direction)) {
      const refined = refineEvent(currentTime, currentState, nextTime, nextState, metric, (state, delta) => rk4Step(state, delta, derivative), direction);
      trajectory.push({ time: refined.time, state: refined.state });
      return { trajectory, event: { time: refined.time, state: refined.state } };
    }

    currentTime = nextTime;
    currentState = nextState;
    currentMetric = nextMetric;
    trajectory.push({ time: currentTime, state: currentState });
  }

  return { trajectory, event: null };
}

function crossesEvent(previous: number, next: number, direction: 1 | -1): boolean {
  if (direction > 0) {
    return previous < 0 && next >= 0;
  }
  return previous > 0 && next <= 0;
}

function refineEvent<T extends number[]>(
  leftTime: number,
  leftState: T,
  rightTime: number,
  rightState: T,
  metric: (state: T) => number,
  advance: (state: T, dt: number) => T,
  direction: 1 | -1,
): TimedState<T> {
  const baseState = leftState.slice() as T;
  let loTime = leftTime;
  let hiTime = rightTime;
  let loMetric = metric(leftState);
  let hiState = rightState.slice() as T;

  for (let index = 0; index < 32; index += 1) {
    const midTime = (loTime + hiTime) / 2;
    const midState = advance(baseState, midTime - leftTime);
    const midMetric = metric(midState);

    if (crossesEvent(loMetric, midMetric, direction)) {
      hiTime = midTime;
      hiState = midState;
    } else {
      loTime = midTime;
      loMetric = midMetric;
    }
  }

  return { time: hiTime, state: hiState };
}

function rk4Step<T extends number[]>(state: T, dt: number, derivative: (value: T) => T): T {
  const k1 = derivative(state);
  const k2 = derivative(addScaled(state, k1, dt / 2));
  const k3 = derivative(addScaled(state, k2, dt / 2));
  const k4 = derivative(addScaled(state, k3, dt));

  return state.map((value, index) => value + (dt / 6) * (k1[index] + 2 * k2[index] + 2 * k3[index] + k4[index])) as T;
}

function addScaled<T extends number[]>(state: T, delta: T, scale: number): T {
  return state.map((value, index) => value + delta[index] * scale) as T;
}

function solve3x3(matrix: number[][], rhs: number[], singularMessage: string): [number, number, number] {
  const det = determinant3x3(matrix);
  if (Math.abs(det) < EPS) {
    throw new Error(singularMessage);
  }

  const det1 = determinant3x3([
    [rhs[0], matrix[0][1], matrix[0][2]],
    [rhs[1], matrix[1][1], matrix[1][2]],
    [rhs[2], matrix[2][1], matrix[2][2]],
  ]);
  const det2 = determinant3x3([
    [matrix[0][0], rhs[0], matrix[0][2]],
    [matrix[1][0], rhs[1], matrix[1][2]],
    [matrix[2][0], rhs[2], matrix[2][2]],
  ]);
  const det3 = determinant3x3([
    [matrix[0][0], matrix[0][1], rhs[0]],
    [matrix[1][0], matrix[1][1], rhs[1]],
    [matrix[2][0], matrix[2][1], rhs[2]],
  ]);

  return [det1 / det, det2 / det, det3 / det];
}

function determinant3x3(matrix: number[][]): number {
  return (
    matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
    matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
    matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
  );
}

function replaceLast<T>(items: T[], value: T): void {
  items[items.length - 1] = value;
}

function safeDiv(numerator: number, denominator: number): number {
  if (Math.abs(denominator) < EPS) {
    denominator = denominator >= 0 ? EPS : -EPS;
  }
  return numerator / denominator;
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function lerp(a: number, b: number, alpha: number): number {
  return a + (b - a) * alpha;
}

void stage1GroundReaction;
