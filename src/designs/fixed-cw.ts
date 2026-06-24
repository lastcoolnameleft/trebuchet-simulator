import type { TrebuchetGeometry, Point } from '../geometry';
import type { SimulationResult, SimulationSample, TrebuchetParams } from '../physics';
import type { TrebuchetDesign, ParameterConfig } from './types';

// ============================================================================
// Fixed Counterweight Trebuchet — derived from designs/fixed-counterweight.yaml
// ============================================================================

const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;
const FIXED_DT = 0.001;
const MAX_TIME = 6.0;
const MAX_FLIGHT_TIME = 20.0;
const EPS = 1e-9;
const EVENT_EPS = 1e-7;
const AIR_DENSITY = 1.225;
const DRAG_COEFFICIENT = 0.47;
const PROJECTILE_DIAMETER = 0.0759;

// Internal params (FCW has no LW, IW3)
interface FCWInternal {
  h: number; LAl: number; LAs: number; LAcg: number;
  mA: number; IA3: number; mW: number;
  LS: number; mP: number;
  releaseAngle: number; releaseAngleRad: number;
  Grav: number; projectileArea: number;
}

// State types: FCW has 2 DOFs (Aq, Sq)
type StageState = [number, number, number, number]; // [Aq, Sq, Aw, Sw]
type FlightState = [number, number, number, number]; // [x, y, vx, vy]

// --- Geometry (from spec) ---

function fcwComputeGeometry(params: TrebuchetParams, sample: SimulationSample): TrebuchetGeometry {
  const { Aq, Sq } = sample;
  const { LAl, LAs, LAcg, LS } = params;

  const cwPoint: Point = {
    x: LAs * Math.sin(Aq),
    y: LAs * Math.cos(Aq),
  };

  return {
    armCg: {
      x: -LAcg * Math.sin(Aq),
      y: -LAcg * Math.cos(Aq),
    },
    counterweightAttach: cwPoint,
    counterweight: cwPoint, // Fixed CW — same as attach point, no hanging rod
    slingAttach: {
      x: -LAl * Math.sin(Aq),
      y: -LAl * Math.cos(Aq),
    },
    projectile: {
      x: -LAl * Math.sin(Aq) - LS * Math.sin(Aq + Sq),
      y: -(LAl * Math.cos(Aq) + LS * Math.cos(Aq + Sq)),
    },
  };
}

// --- Initial conditions (from spec) ---

function initialArmAngle(p: FCWInternal): number {
  const ratio = p.h / p.LAl;
  if (Math.abs(ratio) <= 1.0) return Math.PI - Math.acos(ratio);
  return (14 * Math.PI) / 15;
}

function groundSlingAngle(Aq: number, p: FCWInternal): number {
  const cosTotal = clamp((-p.h - p.LAl * Math.cos(Aq)) / p.LS, -1, 1);
  const total = 2.0 * Math.PI - Math.acos(cosTotal);
  return total - Aq;
}

function sqDot(Aq: number, Sq: number, Aw: number, p: FCWInternal): number {
  const sAs = Math.sin(Aq + Sq);
  return -safeDiv((p.LAl * Math.sin(Aq) + p.LS * sAs) * Aw, p.LS * sAs);
}

// --- Equations of motion (from spec Lagrangian section) ---

function stage2Ode(state: StageState, p: FCWInternal): StageState {
  const [Aq, Sq, Aw, Sw] = state;
  const { LAl, LAs, LAcg, LS, Grav, mA, mW, mP, IA3 } = p;
  const cSq = Math.cos(Sq), sSq = Math.sin(Sq);

  const M11 = IA3 + mW * LAs ** 2 + mA * LAcg ** 2
    + mP * (LAl ** 2 + LS ** 2 + 2 * LAl * LS * cSq);
  const M12 = LS * mP * (LS + LAl * cSq);
  const M22 = mP * LS ** 2;

  const r1 = Grav * LAcg * mA * Math.sin(Aq)
    - Grav * mW * LAs * Math.sin(Aq)
    + Grav * mP * (LAl * Math.sin(Aq) + LS * Math.sin(Aq + Sq))
    - LAl * LS * mP * sSq * (Aw ** 2 - (Aw + Sw) ** 2);
  const r2 = LS * mP * (Grav * Math.sin(Aq + Sq) - LAl * sSq * Aw ** 2);

  const det = M11 * M22 - M12 * M12;
  if (Math.abs(det) < EPS) throw new Error('FCW Stage 2 matrix became singular.');
  const Awd = (r1 * M22 - r2 * M12) / det;
  const Swd = (M11 * r2 - M12 * r1) / det;

  return [Aw, Sw, Awd, Swd];
}

function stage1Ode(state: StageState, p: FCWInternal): StageState {
  const [Aq, _Sq, Aw] = state;
  const { LAl, LAs, LAcg, LS, Grav, mA, mW, mP, IA3 } = p;
  const Sq = groundSlingAngle(Aq, p);
  const sA = Math.sin(Aq), cA = Math.cos(Aq);
  const sSq = Math.sin(Sq), cSq = Math.cos(Sq);
  const sAs = Math.sin(Aq + Sq), cAs = Math.cos(Aq + Sq);
  const Sw = sqDot(Aq, Sq, Aw, p);

  const coupling = safeDiv(cAs * Sw * (Sw + 2 * Aw), sAs);
  const slope = safeDiv(cAs, sAs) + safeDiv(LAl * cA, LS * sAs);
  const accelTerm = coupling + slope * Aw ** 2;

  const M_eff = IA3 + mW * LAs ** 2 + mA * LAcg ** 2
    - mP * LAl ** 2 * (-1 + safeDiv(2 * sA * cSq, sAs))
    + mP * LAl ** 2 * safeDiv(sA ** 2, sAs ** 2);

  const r_eff = Grav * LAcg * mA * sA
    - Grav * mW * LAs * sA
    + LAl * LS * mP * (sSq * (Aw + Sw) ** 2 + cSq * accelTerm)
    + LAl * mP * sA * safeDiv(LAl * sSq * Aw ** 2 - LS * accelTerm, sAs);

  const Awd = r_eff / M_eff;
  const Swd = -safeDiv(cAs * Sw * (Sw + 2 * Aw), sAs)
    - (safeDiv(cAs, sAs) + safeDiv(LAl * cA, LS * sAs)) * Aw ** 2
    - safeDiv((LAl * sA + LS * sAs) * Awd, LS * sAs);

  return [Aw, Sw, Awd, Swd];
}

function flightOde(state: FlightState, p: FCWInternal): FlightState {
  const [, , vx, vy] = state;
  const speed = Math.hypot(vx, vy);
  const dragTerm = (AIR_DENSITY * DRAG_COEFFICIENT * p.projectileArea * speed) / (2 * p.mP);
  return [vx, vy, -dragTerm * vx, -p.Grav - dragTerm * vy];
}

// --- Kinematics ---

function projectileKinematics(state: StageState, p: FCWInternal) {
  const [Aq, Sq, Aw, Sw] = state;
  return {
    x: -p.LAl * Math.sin(Aq) - p.LS * Math.sin(Aq + Sq),
    y: p.h + p.LAl * Math.cos(Aq) + p.LS * Math.cos(Aq + Sq),
    vx: -p.LAl * Math.cos(Aq) * Aw - p.LS * Math.cos(Aq + Sq) * (Aw + Sw),
    vy: -p.LAl * Math.sin(Aq) * Aw - p.LS * Math.sin(Aq + Sq) * (Aw + Sw),
  };
}

// --- Event detection (from spec stages) ---

function liftOffMetric(state: StageState, p: FCWInternal): number {
  const deriv = stage2Ode(state, p);
  const Awd = deriv[2], Swd = deriv[3];
  const [Aq, Sq, Aw, Sw] = state;
  const total = Aq + Sq;
  return p.LAl * Math.cos(Aq) * Aw ** 2 + p.LAl * Math.sin(Aq) * Awd
    + p.LS * Math.cos(total) * (Aw + Sw) ** 2 + p.LS * Math.sin(total) * (Awd + Swd);
}

function releaseEvent(state: StageState, p: FCWInternal): number {
  const { vx, vy } = projectileKinematics(state, p);
  const speed = Math.hypot(vx, vy);
  if (vx <= 0 || speed < 0.5) return -1;
  return Math.atan2(vy, vx) - p.releaseAngleRad;
}

// --- Normalize params ---

function normalizeParams(input: Partial<TrebuchetParams>): FCWInternal {
  const p: FCWInternal = {
    h: 1.524, LAl: 2.0726, LAs: 0.5334, LAcg: 0.7681,
    mA: 4.8307, IA3: 2.7287, mW: 44.4933,
    LS: 2.0828, mP: 0.1497,
    releaseAngle: 45, Grav: 9.81,
    releaseAngleRad: 0, projectileArea: 0,
  };

  // Apply user overrides
  const keys: (keyof FCWInternal)[] = ['h', 'LAl', 'LAs', 'LAcg', 'mA', 'IA3', 'mW', 'LS', 'mP', 'releaseAngle', 'Grav'];
  for (const k of keys) {
    if (k in input && (input as Record<string, number>)[k as string] !== undefined) {
      p[k] = (input as Record<string, number>)[k as string];
    }
  }

  // Auto-compute derived values
  if (!('LAcg' in input)) p.LAcg = (p.LAl - p.LAs) / 2;
  if (!('IA3' in input)) p.IA3 = (1 / 3) * p.mA * (p.LAl + p.LAs) ** 2;

  p.releaseAngleRad = p.releaseAngle * DEG_TO_RAD;
  p.projectileArea = Math.PI * (PROJECTILE_DIAMETER / 2) ** 2;
  return p;
}

function toTrebuchetParams(p: FCWInternal): TrebuchetParams {
  return {
    LAl: p.LAl, LAs: p.LAs, LAcg: p.LAcg,
    LW: 0, LS: p.LS, h: p.h,
    mA: p.mA, mW: p.mW, mP: p.mP,
    IA3: p.IA3, IW3: 0,
    releaseAngle: p.releaseAngle, Grav: p.Grav,
    startArmAngleDeg: 0,
  };
}

// --- Sample construction ---

function makeSample(
  time: number, stage: 'ground' | 'lifted' | 'flight' | 'done',
  Aq: number, Sq: number, Aw: number, Sw: number,
  px: number, py: number, pvx: number, pvy: number,
): SimulationSample {
  return {
    time, stage,
    Aq, Wq: 0, Sq,
    Aw, Ww: 0, Sw,
    projectileX: px, projectileY: py,
    projectileVx: pvx, projectileVy: pvy,
    projectileSpeed: Math.hypot(pvx, pvy),
    releaseAngleNow: Math.atan2(-pvy, pvx) * RAD_TO_DEG,
  };
}

function makeAttachedSample(p: FCWInternal, time: number, stage: 'ground' | 'lifted', state: StageState): SimulationSample {
  let Aq = state[0], Sq = state[1], Aw = state[2], Sw = state[3];
  if (stage === 'ground') {
    Sq = groundSlingAngle(Aq, p);
    Sw = sqDot(Aq, Sq, Aw, p);
  }
  const kin = projectileKinematics([Aq, Sq, Aw, Sw], p);
  return makeSample(time, stage, Aq, Sq, Aw, Sw, kin.x, p.h - kin.y, kin.vx, -kin.vy);
}

// --- Simulation ---

function validateGeometry(p: FCWInternal): void {
  const Aq0 = initialArmAngle(p);
  const reach = Math.abs((-p.h - p.LAl * Math.cos(Aq0)) / p.LS);
  if (reach > 1) {
    throw new Error('Initial geometry cannot place the projectile on the ground. Increase sling length, lower the pivot, or reduce the initial arm angle.');
  }
}

function fcwSimulate(input: Partial<TrebuchetParams>): SimulationResult {
  const p = normalizeParams(input);
  validateGeometry(p);

  const Aq0 = initialArmAngle(p);
  const Sq0 = groundSlingAngle(Aq0, p);
  let state: StageState = [Aq0, Sq0, 0, 0];

  const samples: SimulationSample[] = [];
  const pushSample = (s: SimulationSample) => {
    const last = samples[samples.length - 1];
    if (last && Math.abs(last.time - s.time) < EVENT_EPS) {
      samples[samples.length - 1] = s;
    } else {
      samples.push(s);
    }
  };

  pushSample(makeAttachedSample(p, 0, 'ground', state));

  // Stage 1: Ground constrained (1 DOF)
  let t = 0, liftOffTime = 0;
  const liftMetric = liftOffMetric(state, p);

  if (liftMetric < 0) {
    const liftEvent = integrateUntilEvent(
      state, 0, MAX_TIME, FIXED_DT,
      s => stage1Ode(s, p),
      s => {
        const Sq = groundSlingAngle(s[0], p);
        const Sw = sqDot(s[0], Sq, s[2], p);
        return liftOffMetric([s[0], Sq, s[2], Sw], p);
      },
      1,
    );
    if (!liftEvent.event) throw new Error('Projectile never lifted off the ground.');

    for (const pt of liftEvent.trajectory) {
      pushSample(makeAttachedSample(p, pt.time, 'ground', pt.state));
    }

    const [Aq, , Aw] = liftEvent.event.state;
    const Sq = groundSlingAngle(Aq, p);
    const Sw = sqDot(Aq, Sq, Aw, p);
    state = [Aq, Sq, Aw, Sw];
    t = liftEvent.event.time;
    liftOffTime = t;
    pushSample(makeAttachedSample(p, t, 'ground', state));
  }

  // Stage 2: Lifted (2 DOFs)
  const stage2Run = integrateUntilEvent(
    state, t, MAX_TIME, FIXED_DT,
    s => stage2Ode(s, p),
    s => releaseEvent(s, p),
    -1,
  );
  if (!stage2Run.event) throw new Error('Release angle was never reached.');

  for (const pt of stage2Run.trajectory) {
    pushSample(makeAttachedSample(p, pt.time, 'lifted', pt.state));
  }

  state = stage2Run.event.state;
  const releaseTime = stage2Run.event.time;
  pushSample(makeAttachedSample(p, releaseTime, 'lifted', state));

  // Release kinematics
  const relKin = projectileKinematics(state, p);
  let flight: FlightState = [relKin.x, relKin.y, relKin.vx, relKin.vy];
  const releaseAq = state[0];
  const releaseSq = state[1];
  t = releaseTime;

  // Stage 3: Flight — arm frozen at release position
  let tLand = releaseTime;
  let landed = false;

  while (t < releaseTime + MAX_FLIGHT_TIME - EPS) {
    const dt = Math.min(FIXED_DT, releaseTime + MAX_FLIGHT_TIME - t);
    const nextFlight = rk4Step(flight, dt, s => flightOde(s, p));

    if (flight[1] > 0 && nextFlight[1] <= 0) {
      // Bisect landing
      const baseFlight = flight.slice() as FlightState;
      let lo = 0, hi = dt;
      let hiF = nextFlight;
      for (let i = 0; i < 32; i++) {
        const mid = (lo + hi) / 2;
        const midF = rk4Step(baseFlight, mid, s => flightOde(s, p));
        if (midF[1] <= 0) {
          hi = mid;
          hiF = midF;
        } else {
          lo = mid;
        }
      }
      tLand = t + hi;
      flight = hiF;
      pushSample(makeSample(tLand, 'flight', releaseAq, releaseSq, 0, 0,
        flight[0], p.h - flight[1], flight[2], -flight[3]));
      landed = true;
      break;
    }

    t += dt;
    flight = nextFlight;
    pushSample(makeSample(t, 'flight', releaseAq, releaseSq, 0, 0,
      flight[0], p.h - flight[1], flight[2], -flight[3]));
  }

  if (!landed) throw new Error('Projectile did not land within the flight time limit.');

  // Compute stats
  const initialPx = projectileKinematics([Aq0, Sq0, 0, 0], p).x;
  let peakSpeed = 0, maxHeight = 0;
  for (const s of samples) {
    peakSpeed = Math.max(peakSpeed, s.projectileSpeed);
    const params = toTrebuchetParams(p);
    maxHeight = Math.max(maxHeight, params.h - s.projectileY);
  }

  const releaseSpeed = Math.hypot(relKin.vx, relKin.vy);

  return {
    params: toTrebuchetParams(p),
    samples,
    stats: {
      range: flight[0] - initialPx,
      maxHeight,
      peakSpeed,
      releaseSpeed,
      releaseAngle: Math.atan2(relKin.vy, relKin.vx) * RAD_TO_DEG,
      releaseHeight: relKin.y,
      releaseTime,
      flightTime: tLand - releaseTime,
      totalTime: tLand,
      liftOffTime,
    },
  };
}

function fcwCreateInitialSample(input: Partial<TrebuchetParams>): { params: TrebuchetParams; sample: SimulationSample } {
  const p = normalizeParams(input);
  validateGeometry(p);
  const Aq = initialArmAngle(p);
  const Sq = groundSlingAngle(Aq, p);
  const state: StageState = [Aq, Sq, 0, 0];
  return {
    params: toTrebuchetParams(p),
    sample: makeAttachedSample(p, 0, 'ground', state),
  };
}

// --- Generic infrastructure ---

interface TimedState<T extends number[]> {
  time: number;
  state: T;
}

function rk4Step<T extends number[]>(state: T, dt: number, f: (s: T) => T): T {
  const k1 = f(state);
  const k2 = f(state.map((v, i) => v + dt / 2 * k1[i]) as T);
  const k3 = f(state.map((v, i) => v + dt / 2 * k2[i]) as T);
  const k4 = f(state.map((v, i) => v + dt * k3[i]) as T);
  return state.map((v, i) => v + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])) as T;
}

function integrateUntilEvent<T extends number[]>(
  y0: T, t0: number, tEnd: number, dt: number,
  f: (s: T) => T, metric: (s: T) => number, dir: 1 | -1,
): { trajectory: TimedState<T>[]; event: TimedState<T> | null } {
  const trajectory: TimedState<T>[] = [{ time: t0, state: y0.slice() as T }];
  let state = y0.slice() as T, t = t0, prev = metric(state);
  const crosses = (lo: number, hi: number) =>
    (dir > 0 && lo < 0 && hi >= 0) || (dir < 0 && lo > 0 && hi <= 0);

  while (t < tEnd - EPS) {
    const step = Math.min(dt, tEnd - t);
    const next = rk4Step(state, step, f);
    const cur = metric(next);

    if (crosses(prev, cur)) {
      const base = state.slice() as T;
      let lo = 0, hi = step;
      let loM = prev;
      let hiState = next.slice() as T;
      for (let i = 0; i < 32; i++) {
        const mid = (lo + hi) / 2;
        const midS = rk4Step(base, mid, f);
        const midM = metric(midS);
        if (crosses(loM, midM)) { hi = mid; hiState = midS; }
        else { lo = mid; loM = midM; }
      }
      const eventTime = t + hi;
      trajectory.push({ time: eventTime, state: hiState });
      return { trajectory, event: { time: eventTime, state: hiState } };
    }

    t += step;
    state = next;
    prev = cur;
    trajectory.push({ time: t, state: state.slice() as T });
  }
  return { trajectory, event: null };
}

function safeDiv(n: number, d: number): number {
  if (Math.abs(d) < EPS) d = d >= 0 ? EPS : -EPS;
  return n / d;
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, v));
}

// --- Parameter config (from spec: ui_parameters, no LW or IW3) ---

const parameterConfig: ParameterConfig[] = [
  { id: 'projectileArmLength', physicsKey: 'LAl', label: 'Arm Length (Projectile)', unit: 'm', step: 0.1, min: 0.5, max: 50, default: 2.07 },
  { id: 'counterweightArmLength', physicsKey: 'LAs', label: 'Arm Length (Counterweight)', unit: 'm', step: 0.1, min: 0.1, max: 20, default: 0.533 },
  { id: 'armHeight', physicsKey: 'h', label: 'Pivot Height', unit: 'm', step: 0.1, min: 0.5, max: 40, default: 1.524 },
  { id: 'counterweightMass', physicsKey: 'mW', label: 'Counterweight Mass', unit: 'kg', step: 1, min: 1, max: 5000, default: 44.49 },
  { id: 'projectileMass', physicsKey: 'mP', label: 'Projectile Mass', unit: 'kg', step: 0.01, min: 0.01, max: 200, default: 0.149 },
  { id: 'slingLength', physicsKey: 'LS', label: 'Sling Length', unit: 'm', step: 0.1, min: 0.1, max: 30, default: 2.08 },
  { id: 'armMass', physicsKey: 'mA', label: 'Arm Mass', unit: 'kg', step: 0.5, min: 0.5, max: 500, default: 4.83 },
  { id: 'releaseAngle', physicsKey: 'releaseAngle', label: 'Release Angle', unit: '°', step: 1, min: 10, max: 80, default: 45 },
];

// --- Export design ---

export const fixedCW: TrebuchetDesign = {
  id: 'fixed',
  name: 'Fixed Counterweight',
  parameterConfig,
  simulate: fcwSimulate,
  createInitialSample: fcwCreateInitialSample,
  computeGeometry: fcwComputeGeometry,
};
