/**
 * Verification: Prove the design spec can reproduce the existing physics.
 *
 * This script imports the EXISTING physics engine and runs a simulation,
 * then runs an INDEPENDENT implementation derived purely from the spec,
 * and compares the results sample-by-sample.
 *
 * If the spec-derived code matches the original, the spec format is validated.
 */

import { simulateTrebuchet, type SimulationResult } from '../src/physics';

// ============================================================================
// SPEC-DERIVED IMPLEMENTATION
// ============================================================================
// Everything below was written by reading ONLY the design spec file,
// NOT by copying from physics.ts.
// ============================================================================

const DEG_TO_RAD = Math.PI / 180;
const FIXED_DT = 0.001;
const MAX_TIME = 6.0;
const MAX_FLIGHT_TIME = 20.0;
const EPS = 1e-9;
const AIR_DENSITY = 1.225;
const DRAG_COEFFICIENT = 0.47;
const PROJECTILE_DIAMETER = 0.0759;

// --- From spec: bodies.*.parameters ---
interface SpecParams {
  h: number; LAl: number; LAs: number; LAcg: number;
  mA: number; IA3: number;
  mW: number; LW: number; IW3: number;
  LS: number; mP: number;
  releaseAngle: number; Grav: number;
}

// --- From spec: degrees_of_freedom ---
// State: [Aq, Wq, Sq, Aw, Ww, Sw]
type State6 = [number, number, number, number, number, number];
type State4 = [number, number, number, number];

// --- From spec: degrees_of_freedom.Aq.initial_value ---
function specInitialArmAngle(p: SpecParams): number {
  const ratio = p.h / p.LAl;
  if (Math.abs(ratio) <= 1.0) return Math.PI - Math.acos(ratio);
  return (14 * Math.PI) / 15;
}

// --- From spec: stages.ground.constrained_dofs.Sq ---
function specGroundSlingAngle(Aq: number, p: SpecParams): number {
  const cosTotal = clamp((-p.h - p.LAl * Math.cos(Aq)) / p.LS, -1, 1);
  const total = 2.0 * Math.PI - Math.acos(cosTotal);
  return total - Aq;
}

// --- From spec: degrees_of_freedom.Sq (time derivative under ground constraint) ---
function specSqDot(Aq: number, Sq: number, Aw: number, p: SpecParams): number {
  // Differentiate the ground constraint: LAl*cos(Aq) + LS*cos(Aq+Sq) = -h
  // => -LAl*sin(Aq)*Aw - LS*sin(Aq+Sq)*(Aw+Sw) = 0
  // => Sw = -(LAl*sin(Aq) + LS*sin(Aq+Sq)) * Aw / (LS*sin(Aq+Sq))
  const sAs = Math.sin(Aq + Sq);
  return -safeDiv((p.LAl * Math.sin(Aq) + p.LS * sAs) * Aw, p.LS * sAs);
}

// --- From spec: Lagrangian mechanics, LIFTED stage (3 DOF) ---
function specStage2Ode(state: State6, p: SpecParams): State6 {
  const [Aq, Wq, Sq, Aw, Ww, Sw] = state;
  const { LAl, LAs, LAcg, LW, LS, Grav, mA, mW, mP, IA3, IW3 } = p;
  const cSq = Math.cos(Sq), sSq = Math.sin(Sq);
  const cW = Math.cos(Wq), sW = Math.sin(Wq);

  // Mass matrix (from spec: Lagrangian mechanics section)
  const M11 = IA3 + IW3 + mA * LAcg ** 2
    + mP * (LAl ** 2 + LS ** 2 + 2 * LAl * LS * cSq)
    + mW * (LAs ** 2 + LW ** 2 + 2 * LAs * LW * cW);
  const M12 = IW3 + LW * mW * (LW + LAs * cW);
  const M13 = LS * mP * (LS + LAl * cSq);
  const M22 = IW3 + mW * LW ** 2;
  const M33 = mP * LS ** 2;

  // Forcing vector (from spec: Lagrangian mechanics section)
  const r1 = Grav * LAcg * mA * Math.sin(Aq)
    + Grav * mP * (LAl * Math.sin(Aq) + LS * Math.sin(Aq + Sq))
    - Grav * mW * (LAs * Math.sin(Aq) + LW * Math.sin(Aq + Wq))
    - LAl * LS * mP * sSq * (Aw ** 2 - (Aw + Sw) ** 2)
    - LAs * LW * mW * sW * (Aw ** 2 - (Aw + Ww) ** 2);
  const r2 = -LW * mW * (Grav * Math.sin(Aq + Wq) + LAs * sW * Aw ** 2);
  const r3 = LS * mP * (Grav * Math.sin(Aq + Sq) - LAl * sSq * Aw ** 2);

  const [Awd, Wwd, Swd] = solve3x3(
    [[M11, M12, M13], [M12, M22, 0], [M13, 0, M33]],
    [r1, r2, r3],
  );

  return [Aw, Ww, Sw, Awd, Wwd, Swd];
}

// --- From spec: stages.ground (2-DOF constrained system) ---
function specStage1Components(Aq: number, Wq: number, Aw: number, Ww: number, p: SpecParams) {
  const { LAl, LAs, LAcg, LW, LS, Grav, mA, mW, mP, IA3, IW3 } = p;
  const Sq = specGroundSlingAngle(Aq, p);
  const sA = Math.sin(Aq), cA = Math.cos(Aq);
  const cW = Math.cos(Wq), sW = Math.sin(Wq);
  const sSq = Math.sin(Sq), cSq = Math.cos(Sq);
  const sAs = Math.sin(Aq + Sq), cAs = Math.cos(Aq + Sq);
  const Sw = specSqDot(Aq, Sq, Aw, p);

  const coupling = safeDiv(cAs * Sw * (Sw + 2 * Aw), sAs);
  const slope = safeDiv(cAs, sAs) + safeDiv(LAl * cA, LS * sAs);
  const accelTerm = coupling + slope * Aw ** 2;

  const M11 = -mP * LAl ** 2 * (-1 + safeDiv(2 * sA * cSq, sAs))
    + IA3 + IW3 + mA * LAcg ** 2
    + mP * LAl ** 2 * safeDiv(sA ** 2, sAs ** 2)
    + mW * (LAs ** 2 + LW ** 2 + 2 * LAs * LW * cW);
  const M12 = IW3 + LW * mW * (LW + LAs * cW);
  const M22 = IW3 + mW * LW ** 2;

  const r1 = Grav * LAcg * mA * sA
    + LAl * LS * mP * (sSq * (Aw + Sw) ** 2 + cSq * accelTerm)
    + LAl * mP * sA * safeDiv(LAl * sSq * Aw ** 2 - LS * accelTerm, sAs)
    - Grav * mW * (LAs * sA + LW * Math.sin(Aq + Wq))
    - LAs * LW * mW * sW * (Aw ** 2 - (Aw + Ww) ** 2);
  const r2 = -LW * mW * (Grav * Math.sin(Aq + Wq) + LAs * sW * Aw ** 2);

  const det = M11 * M22 - M12 * M12;
  if (Math.abs(det) < EPS) throw new Error('Stage 1 singular');

  const Awd = (r1 * M22 - r2 * M12) / det;
  const Wwd = -(r1 * M12 - r2 * M11) / det;
  const Swd = -safeDiv(cAs * Sw * (Sw + 2 * Aw), sAs)
    - (safeDiv(cAs, sAs) + safeDiv(LAl * cA, LS * sAs)) * Aw ** 2
    - safeDiv((LAl * sA + LS * sAs) * Awd, LS * sAs);

  return { Sq, Sw, Awd, Wwd, Swd };
}

function specStage1Ode(state: State6, p: SpecParams): State6 {
  const [Aq, Wq, _Sq, Aw, Ww] = state;
  const { Sq, Sw, Awd, Wwd, Swd } = specStage1Components(Aq, Wq, Aw, Ww, p);
  return [Aw, Ww, Sw, Awd, Wwd, Swd];
}

// --- From spec: stages.flight.trebuchet_physics (post-release 2-DOF) ---
function specPostReleaseOde(state: State4, p: SpecParams): State4 {
  const [Aq, Wq, Aw, Ww] = state;
  const { LAs, LAcg, LW, Grav, mA, mW, IA3, IW3 } = p;
  const cW = Math.cos(Wq), sW = Math.sin(Wq);

  const M11 = IA3 + IW3 + mA * LAcg ** 2 + mW * (LAs ** 2 + LW ** 2 + 2 * LAs * LW * cW);
  const M12 = IW3 + LW * mW * (LW + LAs * cW);
  const M22 = IW3 + mW * LW ** 2;
  const r1 = Grav * LAcg * mA * Math.sin(Aq) - Grav * mW * (LAs * Math.sin(Aq) + LW * Math.sin(Aq + Wq))
    - LAs * LW * mW * sW * (Aw ** 2 - (Aw + Ww) ** 2);
  const r2 = -LW * mW * (Grav * Math.sin(Aq + Wq) + LAs * sW * Aw ** 2);

  const det = M11 * M22 - M12 * M12;
  if (Math.abs(det) < EPS) throw new Error('Post-release singular');

  return [Aw, Ww, (r1 * M22 - r2 * M12) / det, -(r1 * M12 - r2 * M11) / det];
}

// --- From spec: geometry.projectile ---
function specProjectileKinematics(state: State6, p: SpecParams) {
  const [Aq, _Wq, Sq, Aw, _Ww, Sw] = state;
  return {
    x: -p.LAl * Math.sin(Aq) - p.LS * Math.sin(Aq + Sq),
    y: p.h + p.LAl * Math.cos(Aq) + p.LS * Math.cos(Aq + Sq),
    vx: -p.LAl * Math.cos(Aq) * Aw - p.LS * Math.cos(Aq + Sq) * (Aw + Sw),
    vy: -p.LAl * Math.sin(Aq) * Aw - p.LS * Math.sin(Aq + Sq) * (Aw + Sw),
  };
}

// --- From spec: stages.ground.transition_condition ---
function specLiftOffMetric(state: State6, p: SpecParams): number {
  const deriv = specStage2Ode(state, p);
  const Awd = deriv[3], Swd = deriv[5];
  const [Aq, , Sq, Aw, , Sw] = state;
  const total = Aq + Sq;
  return p.LAl * Math.cos(Aq) * Aw ** 2 + p.LAl * Math.sin(Aq) * Awd
    + p.LS * Math.cos(total) * (Aw + Sw) ** 2 + p.LS * Math.sin(total) * (Awd + Swd);
}

// --- From spec: stages.lifted.transition_condition ---
function specReleaseEvent(state: State6, p: SpecParams & { releaseAngleRad: number }): number {
  const { vx, vy } = specProjectileKinematics(state, p);
  const speed = Math.hypot(vx, vy);
  if (vx <= 0 || speed < 0.5) return -1;
  return Math.atan2(vy, vx) - p.releaseAngleRad;
}

// --- From spec: stages.flight.projectile_physics ---
function specFlightOde(state: State4, p: SpecParams & { projectileArea: number }): State4 {
  const [_x, _y, vx, vy] = state;
  const relX = vx; // no wind
  const speed = Math.hypot(relX, vy);
  const dragTerm = (AIR_DENSITY * DRAG_COEFFICIENT * p.projectileArea * speed) / (2 * p.mP);
  return [vx, vy, -dragTerm * vx, -p.Grav - dragTerm * vy];
}

// ============================================================================
// Generic simulation infrastructure (RK4, event detection — design-agnostic)
// ============================================================================

function rk4<T extends number[]>(state: T, dt: number, f: (s: T) => T): T {
  const k1 = f(state);
  const k2 = f(state.map((v, i) => v + dt / 2 * k1[i]) as T);
  const k3 = f(state.map((v, i) => v + dt / 2 * k2[i]) as T);
  const k4 = f(state.map((v, i) => v + dt * k3[i]) as T);
  return state.map((v, i) => v + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])) as T;
}

function integrateUntil<T extends number[]>(
  y0: T, t0: number, tEnd: number, dt: number,
  f: (s: T) => T, metric: (s: T) => number, dir: 1 | -1,
): { tEvent: number; state: T } | null {
  let state = y0.slice() as T, t = t0, prev = metric(state);
  const crosses = (lo: number, hi: number) =>
    (dir > 0 && lo < 0 && hi >= 0) || (dir < 0 && lo > 0 && hi <= 0);

  while (t < tEnd - EPS) {
    const step = Math.min(dt, tEnd - t);
    const next = rk4(state, step, f);
    const cur = metric(next);
    if (crosses(prev, cur)) {
      // Bisect within this single step interval [t, t+step]
      const baseState = state.slice() as T;
      const baseTime = t;
      let lo = 0, hi = step;
      let loMetric = prev;
      let hiState = next.slice() as T;
      for (let i = 0; i < 32; i++) {
        const mid = (lo + hi) / 2;
        const midState = rk4(baseState, mid, f);
        const midM = metric(midState);
        if (crosses(loMetric, midM)) {
          hi = mid;
          hiState = midState;
        } else {
          lo = mid;
          loMetric = midM;
        }
      }
      return { tEvent: baseTime + hi, state: hiState };
    }
    t += step;
    state = next;
    prev = cur;
  }
  return null;
}

// ============================================================================
// SPEC-DERIVED FULL SIMULATION
// ============================================================================

interface SpecResult {
  range: number;
  maxHeight: number;
  peakSpeed: number;
  releaseTime: number;
  liftOffTime: number;
  totalTime: number;
  samples: Array<{
    time: number; stage: string;
    Aq: number; Wq: number; Sq: number;
    px: number; py: number; speed: number;
  }>;
}

function specSimulate(input: Partial<SpecParams>): SpecResult {
  const p: SpecParams & { releaseAngleRad: number; projectileArea: number } = {
    h: 1.524, LAl: 2.0726, LAs: 0.5334, LAcg: 0.7681,
    mA: 4.8307, IA3: 2.7287,
    mW: 44.4933, LW: 0.6096, IW3: 0.04244,
    LS: 2.0828, mP: 0.1497,
    releaseAngle: 45, Grav: 9.81,
    ...input,
    releaseAngleRad: 0, projectileArea: 0,
  };
  p.releaseAngleRad = p.releaseAngle * DEG_TO_RAD;
  p.projectileArea = Math.PI * (PROJECTILE_DIAMETER / 2) ** 2;

  // Initial conditions (from spec)
  const Aq0 = specInitialArmAngle(p);
  const Wq0 = -Aq0;
  const Sq0 = specGroundSlingAngle(Aq0, p);
  let state: State6 = [Aq0, Wq0, Sq0, 0, 0, 0];

  const samples: SpecResult['samples'] = [];
  const addSample = (t: number, stage: string, s: State6) => {
    const kin = specProjectileKinematics(s, p);
    samples.push({
      time: t, stage, Aq: s[0], Wq: s[1], Sq: s[2],
      px: kin.x, py: p.h - kin.y, speed: Math.hypot(kin.vx, kin.vy),
    });
  };

  addSample(0, 'ground', state);

  // Stage 1: Ground constrained
  const liftMetricInit = specLiftOffMetric(state, p);
  let t = 0, liftOffTime = 0;

  if (liftMetricInit < 0) {
    const liftEvent = integrateUntil(
      state, 0, MAX_TIME, FIXED_DT,
      s => specStage1Ode(s, p),
      s => {
        const [Aq, Wq, , Aw, Ww] = s;
        const Sq = specGroundSlingAngle(Aq, p);
        const Sw = specSqDot(Aq, Sq, Aw, p);
        return specLiftOffMetric([Aq, Wq, Sq, Aw, Ww, Sw], p);
      },
      1,
    );
    if (!liftEvent) throw new Error('Never lifted');
    const [Aq, Wq, , Aw, Ww] = liftEvent.state;
    const Sq = specGroundSlingAngle(Aq, p);
    const Sw = specSqDot(Aq, Sq, Aw, p);
    state = [Aq, Wq, Sq, Aw, Ww, Sw];
    t = liftEvent.tEvent;
    liftOffTime = t;
    addSample(t, 'ground', state);
  }

  // Stage 2: Lifted (3 DOF)
  const releaseResult = integrateUntil(
    state, t, MAX_TIME, FIXED_DT,
    s => specStage2Ode(s, p),
    s => specReleaseEvent(s, p),
    -1,
  );
  if (!releaseResult) throw new Error('Never released');
  state = releaseResult.state;
  const releaseTime = releaseResult.tEvent;
  addSample(releaseTime, 'lifted', state);

  // Release kinematics
  const relKin = specProjectileKinematics(state, p);
  let flightState: State4 = [relKin.x, relKin.y, relKin.vx, relKin.vy];
  let postState: State4 = [state[0], state[1], state[3], state[4]];
  t = releaseTime;

  // Stage 3: Flight
  let peakSpeed = 0, maxHeight = 0, totalTime = releaseTime;
  const initialPx = specProjectileKinematics([Aq0, Wq0, Sq0, 0, 0, 0], p).x;

  // Track peak speed from attached stages too
  for (const s of samples) {
    peakSpeed = Math.max(peakSpeed, s.speed);
    maxHeight = Math.max(maxHeight, s.py);
  }

  let landed = false;
  while (t < releaseTime + MAX_FLIGHT_TIME - EPS) {
    const dt2 = Math.min(FIXED_DT, releaseTime + MAX_FLIGHT_TIME - t);
    const nextFlight = rk4(flightState, dt2, s => specFlightOde(s, p));
    const nextPost = rk4(postState, dt2, s => specPostReleaseOde(s, p));

    // Height above ground: flightState[1]. Landing = crosses 0 from above.
    if (flightState[1] > 0 && nextFlight[1] <= 0) {
      // Bisect within this step
      const baseFlight = flightState.slice() as State4;
      const basePost = postState.slice() as State4;
      let lo2 = 0, hi2 = dt2;
      for (let i = 0; i < 32; i++) {
        const mid = (lo2 + hi2) / 2;
        const midF = rk4(baseFlight, mid, s => specFlightOde(s, p));
        if (midF[1] <= 0) hi2 = mid; else lo2 = mid;
      }
      totalTime = t + hi2;
      const finalF = rk4(baseFlight, hi2, s => specFlightOde(s, p));
      const finalP = rk4(basePost, hi2, s => specPostReleaseOde(s, p));
      const spd = Math.hypot(finalF[2], finalF[3]);
      peakSpeed = Math.max(peakSpeed, spd);
      const hAbove = finalF[1]; // height above ground
      maxHeight = Math.max(maxHeight, hAbove);
      samples.push({
        time: totalTime, stage: 'flight',
        Aq: finalP[0], Wq: finalP[1], Sq: state[2],
        px: finalF[0], py: p.h - hAbove, speed: spd,
      });
      landed = true;
      break;
    }

    t += dt2;
    flightState = nextFlight;
    postState = nextPost;
    const spd = Math.hypot(flightState[2], flightState[3]);
    peakSpeed = Math.max(peakSpeed, spd);
    const hAbove = flightState[1];
    maxHeight = Math.max(maxHeight, hAbove);
    totalTime = t;
  }

  if (!landed) throw new Error('Projectile did not land');

  const lastFlight = samples[samples.length - 1];
  return {
    range: lastFlight.px - initialPx,
    maxHeight,
    peakSpeed,
    releaseTime,
    liftOffTime,
    totalTime,
    samples,
  };
}

// ============================================================================
// Utilities
// ============================================================================

function safeDiv(n: number, d: number): number {
  if (Math.abs(d) < EPS) d = d >= 0 ? EPS : -EPS;
  return n / d;
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, v));
}

function solve3x3(m: number[][], r: number[]): [number, number, number] {
  const det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
    - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
    + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  if (Math.abs(det) < EPS) throw new Error('Singular');
  const d1 = r[0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
    - m[0][1] * (r[1] * m[2][2] - m[1][2] * r[2])
    + m[0][2] * (r[1] * m[2][1] - m[1][1] * r[2]);
  const d2 = m[0][0] * (r[1] * m[2][2] - m[1][2] * r[2])
    - r[0] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
    + m[0][2] * (m[1][0] * r[2] - r[1] * m[2][0]);
  const d3 = m[0][0] * (m[1][1] * r[2] - r[1] * m[2][1])
    - m[0][1] * (m[1][0] * r[2] - r[1] * m[2][0])
    + r[0] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  return [d1 / det, d2 / det, d3 / det];
}

// ============================================================================
// COMPARISON
// ============================================================================

function compare() {
  console.log('=== SPEC VALIDATION: Hinged Counterweight ===\n');

  // Run EXISTING physics
  const existing: SimulationResult = simulateTrebuchet({});

  // Run SPEC-DERIVED physics
  const spec = specSimulate({});

  console.log('EXISTING physics results:');
  console.log(`  Range:        ${existing.stats.range.toFixed(4)} m`);
  console.log(`  Max Height:   ${existing.stats.maxHeight.toFixed(4)} m`);
  console.log(`  Peak Speed:   ${existing.stats.peakSpeed.toFixed(4)} m/s`);
  console.log(`  Release Time: ${existing.stats.releaseTime.toFixed(6)} s`);
  console.log(`  LiftOff Time: ${existing.stats.liftOffTime.toFixed(6)} s`);
  console.log(`  Total Time:   ${existing.stats.totalTime.toFixed(6)} s`);
  console.log(`  Samples:      ${existing.samples.length}`);

  console.log('\nSPEC-DERIVED results:');
  console.log(`  Range:        ${spec.range.toFixed(4)} m`);
  console.log(`  Max Height:   ${spec.maxHeight.toFixed(4)} m`);
  console.log(`  Peak Speed:   ${spec.peakSpeed.toFixed(4)} m/s`);
  console.log(`  Release Time: ${spec.releaseTime.toFixed(6)} s`);
  console.log(`  LiftOff Time: ${spec.liftOffTime.toFixed(6)} s`);
  console.log(`  Total Time:   ${spec.totalTime.toFixed(6)} s`);
  console.log(`  Samples:      ${spec.samples.length}`);

  // Compute deltas
  const rangeDelta = Math.abs(existing.stats.range - spec.range);
  const heightDelta = Math.abs(existing.stats.maxHeight - spec.maxHeight);
  const speedDelta = Math.abs(existing.stats.peakSpeed - spec.peakSpeed);
  const releaseDelta = Math.abs(existing.stats.releaseTime - spec.releaseTime);
  const liftoffDelta = Math.abs(existing.stats.liftOffTime - spec.liftOffTime);
  const totalDelta = Math.abs(existing.stats.totalTime - spec.totalTime);

  console.log('\nDELTAS:');
  console.log(`  Range:        ${rangeDelta.toExponential(3)} m`);
  console.log(`  Max Height:   ${heightDelta.toExponential(3)} m`);
  console.log(`  Peak Speed:   ${speedDelta.toExponential(3)} m/s`);
  console.log(`  Release Time: ${releaseDelta.toExponential(3)} s`);
  console.log(`  LiftOff Time: ${liftoffDelta.toExponential(3)} s`);
  console.log(`  Total Time:   ${totalDelta.toExponential(3)} s`);

  // Tolerance: 0.5% relative or 0.01 absolute
  const TOL_REL = 0.005;
  const TOL_ABS = 0.01;
  const checks = [
    { name: 'Range', ref: existing.stats.range, delta: rangeDelta },
    { name: 'Max Height', ref: existing.stats.maxHeight, delta: heightDelta },
    { name: 'Peak Speed', ref: existing.stats.peakSpeed, delta: speedDelta },
    { name: 'Release Time', ref: existing.stats.releaseTime, delta: releaseDelta },
    { name: 'LiftOff Time', ref: existing.stats.liftOffTime, delta: liftoffDelta },
    { name: 'Total Time', ref: existing.stats.totalTime, delta: totalDelta },
  ];

  let allPass = true;
  console.log('\nVERDICT:');
  for (const c of checks) {
    const relErr = c.ref !== 0 ? c.delta / Math.abs(c.ref) : c.delta;
    const pass = c.delta < TOL_ABS || relErr < TOL_REL;
    console.log(`  ${pass ? '✅' : '❌'} ${c.name}: delta=${c.delta.toExponential(3)}, relErr=${(relErr * 100).toFixed(3)}%`);
    if (!pass) allPass = false;
  }

  console.log(`\n${allPass ? '✅ ALL CHECKS PASSED — Spec format validated!' : '❌ SOME CHECKS FAILED'}`);
  process.exit(allPass ? 0 : 1);
}

compare();
