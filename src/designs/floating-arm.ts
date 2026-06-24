import type { TrebuchetGeometry } from '../geometry';
import type { SimulationResult, SimulationSample, TrebuchetParams } from '../physics';
import type { TrebuchetDesign, ParameterConfig } from './types';

// ============================================================================
// Floating Arm Trebuchet — 5DOF Constrained Dynamics
// ============================================================================
// Based on the Constans/Rowan University 5DOF floating arm model.
//
// DOFs: h (CW height), x (proj x), y (proj y), θ (arm angle), ψ (sling angle)
// The CW drops vertically along rails, attached to one end of the arm.
// A fixed pin on the base passes through a slot in the arm, creating the fulcrum.
// The sling hangs from the arm tip (opposite end from CW).
//
// Key geometry:
//   - CW attached at one end of arm, drops straight down (h DOF)
//   - Arm pivots around a point at distance D from CW end (rides in vertical rails)
//   - Fixed pin at (W horizontal, H vertical from base bottom) constrains arm via slot
//   - Sling of length L3 connects arm tip to projectile
//   - Projectile starts on a ramp at angle λ
//
// 4 Configurations:
//   Config 1: Fixed pin in arm slot + projectile on ramp
//   Config 2: Arm pivot in fixed slot (pin reached) + projectile on ramp
//   Config 3: Fixed pin in arm slot + projectile above ramp
//   Config 4: Arm pivot in fixed slot + projectile above ramp
//
// Coordinate system: origin at base bottom center, x positive LEFT (throw dir), y UP
// ============================================================================

const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;
const FIXED_DT = 0.001;
const MAX_TIME = 8.0;
const MAX_FLIGHT_TIME = 20.0;
const GRAV = 9.81;
const AIR_DENSITY = 1.225;
const DRAG_COEFFICIENT = 0.47;
const PROJECTILE_DIAMETER = 0.0759;

// FAT-specific parameters
interface FATParams {
  m1: number;      // CW mass (kg)
  m2: number;      // Projectile mass (kg)
  L2: number;      // Arm length from CW end to hook (m)
  D: number;       // Distance from CW end to arm pivot (m)
  L3: number;      // Sling length (m)
  h0: number;      // Starting height of CW / base height (m)
  H: number;       // Height of fixed pin from base bottom (m)
  W: number;       // Horizontal distance from rails to fixed pin (m)
  armWidth: number; // Arm width (m)
  armThickness: number; // Arm thickness (m)
  armDensity: number;   // Arm density (kg/m³)
  beta: number;    // Hook/release angle (rad) — sling releases when angle reaches this
  lambda: number;  // Ramp angle (rad)
  Ib: number;      // Arm moment of inertia about CM (computed)
  mb: number;      // Arm mass (computed)
}

// State: [h, x, y, theta, psi, hdot, xdot, ydot, thetadot, psidot]
type FATState = [number, number, number, number, number, number, number, number, number, number];
type FlightState = [number, number, number, number]; // [x, y, vx, vy]

interface TimedState<T extends number[]> {
  time: number;
  state: T;
}

// ============================================================================
// Linear algebra for augmented constraint systems
// ============================================================================

function solveLinearSystem(A: number[][], b: number[]): number[] {
  const n = A.length;
  // Gaussian elimination with partial pivoting
  const M = A.map((row, i) => [...row, b[i]]);

  for (let col = 0; col < n; col++) {
    // Partial pivoting
    let maxRow = col;
    let maxVal = Math.abs(M[col][col]);
    for (let row = col + 1; row < n; row++) {
      if (Math.abs(M[row][col]) > maxVal) {
        maxVal = Math.abs(M[row][col]);
        maxRow = row;
      }
    }
    if (maxRow !== col) {
      [M[col], M[maxRow]] = [M[maxRow], M[col]];
    }

    const pivot = M[col][col];
    if (Math.abs(pivot) < 1e-14) continue;

    for (let row = col + 1; row < n; row++) {
      const factor = M[row][col] / pivot;
      for (let j = col; j <= n; j++) {
        M[row][j] -= factor * M[col][j];
      }
    }
  }

  // Back substitution
  const x = new Array(n).fill(0);
  for (let row = n - 1; row >= 0; row--) {
    let sum = M[row][n];
    for (let col = row + 1; col < n; col++) {
      sum -= M[row][col] * x[col];
    }
    x[row] = Math.abs(M[row][row]) > 1e-14 ? sum / M[row][row] : 0;
  }
  return x;
}

// ============================================================================
// Physics: Build and solve augmented system for each configuration
// ============================================================================

function solveConfig(
  p: FATParams,
  state: FATState,
  config: 1 | 2 | 3 | 4,
): { accel: number[]; lambda_forces: number[] } {
  const [h, x, y, theta, psi, hdot, xdot, ydot, thetadot, psidot] = state;
  const { m1, m2, L2, D, H, W, L3, Ib, lambda } = p;

  const CQ = Math.cos(theta);
  const SQ = Math.sin(theta);
  const CS = Math.cos(psi);
  const SS = Math.sin(psi);
  const CL = Math.cos(lambda);
  const SL = Math.sin(lambda);
  const q2 = thetadot * thetadot;
  const s2 = psidot * psidot;

  let A: number[][];
  let b: number[];

  if (config === 1) {
    // Fixed pin in arm slot + projectile on ramp (9×9)
    // Unknowns: [h_ddot, x_ddot, y_ddot, θ_ddot, ψ_ddot, λ1, λ2, λ3, λ4]
    // Constraints: sling_x, sling_y, fixed_pin_slot, ramp
    A = [
      [m1, 0, 0, 0, 0, 0, 1, -CQ, 0],
      [0, m2, 0, 0, 0, -1, 0, 0, SL],
      [0, 0, m2, 0, 0, 0, -1, 0, CL],
      [0, 0, 0, Ib, 0, L2 * SQ, -L2 * CQ, W * CQ - (H - h) * SQ, 0],
      [0, 0, 0, 0, 0, -L3 * SS, L3 * CS, 0, 0],
      [0, -1, 0, L2 * SQ, -L3 * SS, 0, 0, 0, 0],
      [1, 0, -1, -L2 * CQ, L3 * CS, 0, 0, 0, 0],
      [-CQ, 0, 0, W * CQ - (H - h) * SQ, 0, 0, 0, 0, 0],
      [0, SL, CL, 0, 0, 0, 0, 0, 0],
    ];
    b = [
      -m1 * GRAV,
      0,
      -m2 * GRAV,
      0,
      0,
      -L2 * q2 * CQ + L3 * s2 * CS,
      -L2 * q2 * SQ + L3 * s2 * SS,
      -2 * hdot * thetadot * SQ + q2 * (W * SQ + (H - h) * CQ),
      0,
    ];
  } else if (config === 2) {
    // Arm pivot in fixed slot + projectile on ramp (9×9)
    // Pin constraint: h - D*sin(θ) - H = 0
    A = [
      [m1, 0, 0, 0, 0, 0, 1, 1, 0],
      [0, m2, 0, 0, 0, -1, 0, 0, SL],
      [0, 0, m2, 0, 0, 0, -1, 0, CL],
      [0, 0, 0, Ib, 0, L2 * SQ, -L2 * CQ, -D * CQ, 0],
      [0, 0, 0, 0, 0, -L3 * SS, L3 * CS, 0, 0],
      [0, -1, 0, L2 * SQ, -L3 * SS, 0, 0, 0, 0],
      [1, 0, -1, -L2 * CQ, L3 * CS, 0, 0, 0, 0],
      [1, 0, 0, -D * CQ, 0, 0, 0, 0, 0],
      [0, SL, CL, 0, 0, 0, 0, 0, 0],
    ];
    b = [
      -m1 * GRAV,
      0,
      -m2 * GRAV,
      0,
      0,
      -L2 * q2 * CQ + L3 * s2 * CS,
      -L2 * q2 * SQ + L3 * s2 * SS,
      -D * q2 * SQ,
      0,
    ];
  } else if (config === 3) {
    // Fixed pin in arm slot + projectile above ramp (8×8)
    A = [
      [m1, 0, 0, 0, 0, 0, 1, -CQ],
      [0, m2, 0, 0, 0, -1, 0, 0],
      [0, 0, m2, 0, 0, 0, -1, 0],
      [0, 0, 0, Ib, 0, L2 * SQ, -L2 * CQ, W * CQ - (H - h) * SQ],
      [0, 0, 0, 0, 0, -L3 * SS, L3 * CS, 0],
      [0, -1, 0, L2 * SQ, -L3 * SS, 0, 0, 0],
      [1, 0, -1, -L2 * CQ, L3 * CS, 0, 0, 0],
      [-CQ, 0, 0, W * CQ - (H - h) * SQ, 0, 0, 0, 0],
    ];
    b = [
      -m1 * GRAV,
      0,
      -m2 * GRAV,
      0,
      0,
      -L2 * q2 * CQ + L3 * s2 * CS,
      -L2 * q2 * SQ + L3 * s2 * SS,
      -2 * hdot * thetadot * SQ + q2 * (W * SQ + (H - h) * CQ),
    ];
  } else {
    // Config 4: Arm pivot in fixed slot + projectile above ramp (8×8)
    A = [
      [m1, 0, 0, 0, 0, 0, 1, 1],
      [0, m2, 0, 0, 0, -1, 0, 0],
      [0, 0, m2, 0, 0, 0, -1, 0],
      [0, 0, 0, Ib, 0, L2 * SQ, -L2 * CQ, -D * CQ],
      [0, 0, 0, 0, 0, -L3 * SS, L3 * CS, 0],
      [0, -1, 0, L2 * SQ, -L3 * SS, 0, 0, 0],
      [1, 0, -1, -L2 * CQ, L3 * CS, 0, 0, 0],
      [1, 0, 0, -D * CQ, 0, 0, 0, 0],
    ];
    b = [
      -m1 * GRAV,
      0,
      -m2 * GRAV,
      0,
      0,
      -L2 * q2 * CQ + L3 * s2 * CS,
      -L2 * q2 * SQ + L3 * s2 * SS,
      -D * q2 * SQ,
    ];
  }

  const solution = solveLinearSystem(A, b);
  const nDOF = 5;
  return {
    accel: solution.slice(0, nDOF),
    lambda_forces: solution.slice(nDOF),
  };
}

// ============================================================================
// Constraint enforcement via Newton-Raphson iteration
// ============================================================================

function enforceConstraints(state: FATState, p: FATParams, config: 1 | 2 | 3 | 4): FATState {
  const { L2, L3, D, H, W, lambda } = p;
  const CL = Math.cos(lambda);
  const SL = Math.sin(lambda);
  const tol = 1e-10;

  let [h, x, y, theta, psi, hdot, xdot, ydot, thetadot, psidot] = state;

  for (let iter = 0; iter < 100; iter++) {
    let Phi: number[];
    let Jac: number[][];

    if (config === 1) {
      Phi = [
        -L2 * Math.cos(theta) - x + L3 * Math.cos(psi),
        h - L2 * Math.sin(theta) - y + L3 * Math.sin(psi),
        W * Math.sin(theta) + (H - h) * Math.cos(theta),
        x * SL + y * CL,
      ];
      if (Phi.reduce((s, v) => s + v * v, 0) < tol) break;
      Jac = [
        [0, -1, 0, L2 * Math.sin(theta), -L3 * Math.sin(psi)],
        [1, 0, -1, -L2 * Math.cos(theta), L3 * Math.cos(psi)],
        [-Math.cos(theta), 0, 0, W * Math.cos(theta) - (H - h) * Math.sin(theta), 0],
        [0, SL, CL, 0, 0],
      ];
    } else if (config === 2) {
      Phi = [
        -L2 * Math.cos(theta) - x + L3 * Math.cos(psi),
        h - L2 * Math.sin(theta) - y + L3 * Math.sin(psi),
        h - D * Math.sin(theta) - H,
        x * SL + y * CL,
      ];
      if (Phi.reduce((s, v) => s + v * v, 0) < tol) break;
      Jac = [
        [0, -1, 0, L2 * Math.sin(theta), -L3 * Math.sin(psi)],
        [1, 0, -1, -L2 * Math.cos(theta), L3 * Math.cos(psi)],
        [1, 0, 0, -D * Math.cos(theta), 0],
        [0, SL, CL, 0, 0],
      ];
    } else if (config === 3) {
      Phi = [
        -L2 * Math.cos(theta) - x + L3 * Math.cos(psi),
        h - L2 * Math.sin(theta) - y + L3 * Math.sin(psi),
        W * Math.sin(theta) + (H - h) * Math.cos(theta),
      ];
      if (Phi.reduce((s, v) => s + v * v, 0) < tol) break;
      Jac = [
        [0, -1, 0, L2 * Math.sin(theta), -L3 * Math.sin(psi)],
        [1, 0, -1, -L2 * Math.cos(theta), L3 * Math.cos(psi)],
        [-Math.cos(theta), 0, 0, W * Math.cos(theta) - (H - h) * Math.sin(theta), 0],
      ];
    } else {
      Phi = [
        -L2 * Math.cos(theta) - x + L3 * Math.cos(psi),
        h - L2 * Math.sin(theta) - y + L3 * Math.sin(psi),
        h - D * Math.sin(theta) - H,
      ];
      if (Phi.reduce((s, v) => s + v * v, 0) < tol) break;
      Jac = [
        [0, -1, 0, L2 * Math.sin(theta), -L3 * Math.sin(psi)],
        [1, 0, -1, -L2 * Math.cos(theta), L3 * Math.cos(psi)],
        [1, 0, 0, -D * Math.cos(theta), 0],
      ];
    }

    // Solve Jac * dq = Phi (for correction dq = [dh, dx, dy, dtheta, dpsi])
    // Use subset of variables depending on config constraints
    // For 4-constraint configs, solve 4×4 with variables [h, x, y, psi] (theta fixed by arm inertia)
    // For 3-constraint configs, solve 3×3 with variables [h, x, y] (theta, psi from dynamics)
    // Actually reference iterates on [h, x, y, psi] for configs 1&2 and [h, x, y] for configs 3&4

    if (config === 1 || config === 2) {
      // 4 constraints, 4 unknowns: [h, x, y, psi]
      // Jac columns: [h, x, y, theta, psi] — take columns [0,1,2,4]
      const J4 = Jac.map(row => [row[0], row[1], row[2], row[4]]);
      const correction = solveLinearSystem(J4, Phi);
      h -= correction[0];
      x -= correction[1];
      y -= correction[2];
      psi -= correction[3];
    } else {
      // 3 constraints, 3 unknowns: [h, x, y]
      const J3 = Jac.map(row => [row[0], row[1], row[2]]);
      const correction = solveLinearSystem(J3, Phi);
      h -= correction[0];
      x -= correction[1];
      y -= correction[2];
    }
  }

  return [h, x, y, theta, psi, hdot, xdot, ydot, thetadot, psidot];
}

// ============================================================================
// Time stepping (Euler integration + constraint stabilization)
// ============================================================================

function stepFAT(state: FATState, p: FATParams, dt: number, config: 1 | 2 | 3 | 4): {
  newState: FATState;
  rampNormal: number; // Normal force on ramp (>0 means on ramp, used for liftoff detection)
} {
  const { accel, lambda_forces } = solveConfig(p, state, config);
  const [h, x, y, theta, psi, hdot, xdot, ydot, thetadot, psidot] = state;

  // Euler integration on velocities
  const newHdot = hdot + accel[0] * dt;
  const newXdot = xdot + accel[1] * dt;
  const newYdot = ydot + accel[2] * dt;
  const newThetadot = thetadot + accel[3] * dt;
  const newPsidot = psidot + accel[4] * dt;

  // Euler integration on positions
  const newH = h + newHdot * dt;
  const newX = x + newXdot * dt;
  const newY = y + newYdot * dt;
  const newTheta = theta + newThetadot * dt;
  const newPsi = psi + newPsidot * dt;

  let newState: FATState = [newH, newX, newY, newTheta, newPsi, newHdot, newXdot, newYdot, newThetadot, newPsidot];

  // Enforce constraints (position level stabilization)
  newState = enforceConstraints(newState, p, config);

  // CW ground stop: CW cannot drop below h=0 (ground level)
  // Applied after constraint enforcement to prevent override
  if (newState[0] < 0) {
    newState[0] = 0;       // clamp h at ground
    if (newState[5] < 0) newState[5] = 0;  // zero downward hdot (inelastic stop)
  }

  // Ramp normal force is the last lambda in configs 1 and 2
  // For configs 3 and 4, ramp is already off
  const rampNormal = (config === 1 || config === 2) ? lambda_forces[lambda_forces.length - 1] : -1;

  return { newState, rampNormal };
}

// ============================================================================
// Configuration detection
// ============================================================================

function detectConfig(state: FATState, p: FATParams, currentConfig: 1 | 2 | 3 | 4): 1 | 2 | 3 | 4 {
  const [h, , , theta] = state;
  const { D, H } = p;

  // Pin transition: when h - D*sin(θ) reaches H, arm pivot enters fixed slot
  const pinInSlot = (h - D * Math.sin(theta)) <= H;
  const onRamp = currentConfig === 1 || currentConfig === 2;

  if (onRamp) {
    return pinInSlot ? 2 : 1;
  } else {
    return pinInSlot ? 4 : 3;
  }
}

// ============================================================================
// Release detection
// ============================================================================

function isReleased(state: FATState, p: FATParams): boolean {
  const [, , , theta, psi] = state;
  // Release when: -psi - π + theta >= beta
  // This is the hook angle condition from the reference
  return (-psi - Math.PI + theta) >= p.beta;
}

// ============================================================================
// Main simulation
// ============================================================================

function fatSimulate(input: Partial<TrebuchetParams>): SimulationResult {
  const p = buildFATParams(input);

  // Initial conditions
  const theta0 = Math.atan((p.h0 - p.H) / p.W);
  let psi0: number;
  const testVal = (p.L2 * Math.sin(p.lambda + theta0) - p.h0 * Math.cos(p.lambda)) / p.L3;
  let onRamp: boolean;
  if (testVal > 1 || testVal < -1) {
    psi0 = -Math.PI / 2;
    onRamp = false;
  } else {
    psi0 = Math.asin(testVal) - p.lambda;
    onRamp = true;
  }

  // Initial projectile position (from arm tip + sling)
  const x0 = -p.L2 * Math.cos(theta0) + p.L3 * Math.cos(psi0);
  const y0 = p.h0 - p.L2 * Math.sin(theta0) + p.L3 * Math.sin(psi0);

  let state: FATState = [p.h0, x0, y0, theta0, psi0, 0, 0, 0, 0, 0];

  // Detect initial config
  const pinInSlot0 = (p.h0 - p.D * Math.sin(theta0)) <= p.H;
  let config: 1 | 2 | 3 | 4 = onRamp ? (pinInSlot0 ? 2 : 1) : (pinInSlot0 ? 4 : 3);

  const samples: SimulationSample[] = [];
  let t = 0;
  let launched = false;
  let releaseTime = 0;
  let releaseState: FATState | null = null;

  // Record initial sample
  samples.push(makeSample(state, p, t, 'ground'));

  // Pre-launch simulation
  while (t < MAX_TIME && !launched) {
    const { newState, rampNormal } = stepFAT(state, p, FIXED_DT, config);
    t += FIXED_DT;
    state = newState;

    // Check ramp liftoff
    if ((config === 1 || config === 2) && rampNormal >= 0) {
      onRamp = false;
      config = detectConfig(state, p, config === 1 ? 3 : 4);
    }

    // Update config (pin transition)
    config = detectConfig(state, p, config);

    // Check release
    if (isReleased(state, p)) {
      launched = true;
      releaseTime = t;
      releaseState = state;
    }

    const stage = launched ? 'lifted' : (onRamp ? 'ground' : 'lifted');
    samples.push(makeSample(state, p, t, stage));
  }

  // Post-release: projectile in flight
  let flightSamples: SimulationSample[] = [];
  let range = 0;
  let maxHeight = 0;
  let peakSpeed = 0;
  let releaseSpeed = 0;
  let releaseHeight = 0;
  let flightTime = 0;
  let releaseVx = 0;
  let releaseVy = 0;

  if (releaseState) {
    const [h, x, y, theta, psi, hdot, xdot, ydot, thetadot, psidot] = releaseState;

    // Projectile velocity at release (from dynamics)
    releaseVx = xdot;
    releaseVy = ydot;
    releaseSpeed = Math.sqrt(releaseVx * releaseVx + releaseVy * releaseVy);
    releaseHeight = y;

    // Flight phase with drag
    const projArea = Math.PI * (PROJECTILE_DIAMETER / 2) ** 2;
    let flightState: FlightState = [x, y, releaseVx, releaseVy];
    let ft = 0;

    while (ft < MAX_FLIGHT_TIME) {
      const [fx, fy, fvx, fvy] = flightState;
      const speed = Math.sqrt(fvx * fvx + fvy * fvy);
      const dragForce = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * projArea * speed;
      const ax = speed > 0 ? -dragForce * fvx / (speed * p.m2) : 0;
      const ay = -GRAV + (speed > 0 ? -dragForce * fvy / (speed * p.m2) : 0);

      const newFx = fx + fvx * FIXED_DT;
      const newFy = fy + fvy * FIXED_DT;
      const newFvx = fvx + ax * FIXED_DT;
      const newFvy = fvy + ay * FIXED_DT;

      flightState = [newFx, newFy, newFvx, newFvy];
      ft += FIXED_DT;

      // Hit ground (y <= 0) — stop before recording this sample
      if (newFy <= 0) {
        range = Math.abs(newFx - x);
        flightTime = ft;
        break;
      }

      if (newFy > maxHeight) maxHeight = newFy;
      peakSpeed = Math.max(peakSpeed, Math.sqrt(newFvx * newFvx + newFvy * newFvy));

      // Record every 10 steps
      if (Math.round(ft / FIXED_DT) % 10 === 0) {
        flightSamples.push(makeFlightSample(flightState, p, releaseTime + ft, releaseState));
      }
    }
  }

  // Combine samples
  const allSamples = [...samples, ...flightSamples];

  // Mark flight samples
  flightSamples.forEach(s => { s.stage = 'flight'; });

  const totalTime = releaseTime + flightTime;
  const stats = {
    range,
    maxHeight,
    peakSpeed: Math.max(peakSpeed, releaseSpeed),
    releaseSpeed,
    releaseAngle: releaseVy !== 0 || releaseVx !== 0
      ? Math.atan2(releaseVy, Math.abs(releaseVx)) * RAD_TO_DEG : 0,
    releaseHeight,
    releaseTime,
    flightTime,
    totalTime,
    liftOffTime: 0, // FAT doesn't have a distinct liftoff time in same sense
  };

  return {
    params: toTrebuchetParams(input, p),
    samples: allSamples,
    stats,
  };
}

// ============================================================================
// Convert state to SimulationSample
// ============================================================================

function makeSample(state: FATState, p: FATParams, t: number, stage: string): SimulationSample {
  const [h, x, y, theta, psi, hdot, xdot, ydot, thetadot, psidot] = state;

  // Map to renderer coordinates:
  // Our renderer: y-DOWN from frame top, x positive to the right
  // Reference model: origin at base bottom, x positive LEFT, y UP
  // Renderer: origin at frame top (= base top = h0 height), y positive DOWN
  // projectileX in renderer = distance from pivot horizontally (reference x is throw dir)
  // projectileY in renderer = h0 - y (convert y-up to y-down from top)

  return {
    time: t,
    stage: stage as any,
    Aq: theta,
    Wq: h,        // Store CW height in Wq slot
    Sq: psi,
    Aw: thetadot,
    Ww: hdot,     // Store hdot in Ww slot
    Sw: psidot,
    projectileX: x,      // Horizontal distance (throw direction)
    projectileY: p.h0 - y,  // Convert y-UP to renderer y-DOWN from top
    projectileVx: xdot,
    projectileVy: -ydot,    // Flip for renderer
    projectileSpeed: Math.sqrt(xdot * xdot + ydot * ydot),
    releaseAngleNow: Math.atan2(ydot, Math.abs(xdot)) * RAD_TO_DEG,
  };
}

function makeFlightSample(fs: FlightState, p: FATParams, t: number, releaseState: FATState): SimulationSample {
  const [fx, fy, fvx, fvy] = fs;
  const [h, , , theta, psi, hdot, , , thetadot, psidot] = releaseState;

  return {
    time: t,
    stage: 'flight',
    Aq: theta,     // Frozen at release
    Wq: h,
    Sq: psi,
    Aw: 0,
    Ww: 0,
    Sw: 0,
    projectileX: fx,
    projectileY: p.h0 - fy,
    projectileVx: fvx,
    projectileVy: -fvy,
    projectileSpeed: Math.sqrt(fvx * fvx + fvy * fvy),
    releaseAngleNow: Math.atan2(fvy, Math.abs(fvx)) * RAD_TO_DEG,
  };
}

// ============================================================================
// Geometry function for renderer
// ============================================================================

function fatComputeGeometry(params: TrebuchetParams, sample: SimulationSample): TrebuchetGeometry {
  const h = sample.Wq;       // CW height from base bottom (y-UP)
  const theta = sample.Aq;   // Arm angle
  const psi = sample.Sq;     // Sling angle
  const h0 = params.h;       // Base height
  const L2 = params.LAl;     // Arm length (CW to hook)
  const L3 = params.LS;      // Sling length
  const D = params.LAs;      // CW to pivot distance

  // Convert to renderer coords (y-DOWN from frame top which is at h0)
  // In reference: y=0 is base bottom, y=h0 is top
  // In renderer: y=0 is frame top, y=h0 is ground (y-DOWN)
  // So renderer_y = h0 - physics_y

  // CW position: at (0, h) in physics → renderer: (0, h0 - h)
  const cwRendererY = h0 - h;

  // Arm CW-end is at (0, h) in physics
  // Arm hook-end is at (-L2*cos(θ), h - L2*sin(θ)) in physics
  // Arm pivot (at distance D from CW end) is at (-D*cos(θ), h - D*sin(θ))

  // In renderer coords (y flipped, x flipped for left=positive→right=positive):
  // Actually our renderer expects x with throw direction = positive right
  // Reference has x positive LEFT. For the renderer, let's keep x as-is (positive = throw dir)

  const armTipX = -L2 * Math.cos(theta);
  const armTipPhysY = h - L2 * Math.sin(theta);
  const armTipRendererY = h0 - armTipPhysY;

  const pivotX = -D * Math.cos(theta);
  const pivotPhysY = h - D * Math.sin(theta);
  const pivotRendererY = h0 - pivotPhysY;

  // Projectile position
  const projX = armTipX + L3 * Math.cos(psi);
  const projPhysY = armTipPhysY + L3 * Math.sin(psi);
  const projRendererY = h0 - projPhysY;

  // Arm CG (midpoint of arm for uniform beam, at L2/2 from CW end)
  const armCgX = -(L2 / 2) * Math.cos(theta);
  const armCgPhysY = h - (L2 / 2) * Math.sin(theta);
  const armCgRendererY = h0 - armCgPhysY;

  // Track data for rendering
  const W = (params as any).pinDistance ?? 0.2;
  const H = (params as any).pinHeight ?? 0.2;
  const pinRendererY = h0 - H;

  return {
    pivotY: pivotRendererY,
    armCg: { x: armCgX, y: armCgRendererY },
    counterweightAttach: { x: 0, y: cwRendererY },
    counterweight: { x: 0, y: cwRendererY },
    slingAttach: { x: armTipX, y: armTipRendererY },
    projectile: { x: projX, y: projRendererY },
    tracks: {
      vertical: { x: 0, yTop: 0, yBottom: h0 },
      horizontal: { x: -W, y: pinRendererY, length: L2 * 0.8 },
    },
  };
}

// ============================================================================
// Parameter building
// ============================================================================

function buildFATParams(input: Partial<TrebuchetParams>): FATParams {
  const m1 = input.mW ?? 10;          // CW mass
  const m2 = input.mP ?? 0.1;         // Projectile mass
  const L2 = input.LAl ?? 0.4;        // Arm length (CW to hook)
  const D = input.LAs ?? 0.15;        // CW to pivot distance
  const L3 = input.LS ?? 0.2;         // Sling length
  const h0 = input.h ?? 0.4;          // Base height
  const H = (input as any).pinHeight ?? 0.2;     // Fixed pin height
  const W = (input as any).pinDistance ?? 0.2;    // Horizontal pin distance
  const armWidth = (input as any).armWidth ?? 0.04;
  const armThickness = (input as any).armThickness ?? 0.01;
  const armDensity = (input as any).armDensity ?? 600;
  const beta = (input.releaseAngle ?? -65) * DEG_TO_RAD; // Hook angle (in deg from user, convert to rad)
  const lambda = ((input as any).rampAngle ?? 10) * DEG_TO_RAD; // Ramp angle

  const mb = armDensity * armThickness * armWidth * L2;
  const Ib = (mb / 12) * (L2 * L2 + armWidth * armWidth);

  return { m1, m2, L2, D, L3, h0, H, W, armWidth, armThickness, armDensity, beta, lambda, Ib, mb };
}

function toTrebuchetParams(input: Partial<TrebuchetParams>, p: FATParams): TrebuchetParams {
  return {
    LAl: p.L2,
    LAs: p.D,
    LAcg: p.L2 / 2,
    LW: 0,
    LS: p.L3,
    h: p.h0,
    mA: p.mb,
    mW: p.m1,
    mP: p.m2,
    IA3: p.Ib,
    IW3: 0,
    releaseAngle: p.beta * RAD_TO_DEG,
    Grav: GRAV,
    startArmAngleDeg: Math.atan((p.h0 - p.H) / p.W) * RAD_TO_DEG,
    // FAT-specific extra params (cast through)
    ...(input as any).pinHeight !== undefined ? { pinHeight: (input as any).pinHeight } : { pinHeight: p.H },
    ...(input as any).pinDistance !== undefined ? { pinDistance: (input as any).pinDistance } : { pinDistance: p.W },
    ...(input as any).rampAngle !== undefined ? { rampAngle: (input as any).rampAngle } : { rampAngle: p.lambda * RAD_TO_DEG },
    ...(input as any).armWidth !== undefined ? { armWidth: (input as any).armWidth } : { armWidth: p.armWidth },
    ...(input as any).armThickness !== undefined ? { armThickness: (input as any).armThickness } : { armThickness: p.armThickness },
    ...(input as any).armDensity !== undefined ? { armDensity: (input as any).armDensity } : { armDensity: p.armDensity },
  } as any;
}

// ============================================================================
// Initial sample (for static display before simulation)
// ============================================================================

function fatCreateInitialSample(input: Partial<TrebuchetParams>): { params: TrebuchetParams; sample: SimulationSample } {
  const p = buildFATParams(input);
  const theta0 = Math.atan((p.h0 - p.H) / p.W);
  let psi0: number;
  const testVal = (p.L2 * Math.sin(p.lambda + theta0) - p.h0 * Math.cos(p.lambda)) / p.L3;
  if (testVal > 1 || testVal < -1) {
    psi0 = -Math.PI / 2;
  } else {
    psi0 = Math.asin(testVal) - p.lambda;
  }

  const x0 = -p.L2 * Math.cos(theta0) + p.L3 * Math.cos(psi0);
  const y0 = p.h0 - p.L2 * Math.sin(theta0) + p.L3 * Math.sin(psi0);

  const state: FATState = [p.h0, x0, y0, theta0, psi0, 0, 0, 0, 0, 0];
  return {
    params: toTrebuchetParams(input, p),
    sample: makeSample(state, p, 0, 'ground'),
  };
}

// ============================================================================
// Parameter config
// ============================================================================

const parameterConfig: ParameterConfig[] = [
  { id: 'armLength', physicsKey: 'LAl', label: 'Arm Length (CW to Hook)', unit: 'm', step: 0.05, min: 0.1, max: 2, default: 0.4 },
  { id: 'pivotDistance', physicsKey: 'LAs', label: 'CW to Pivot Distance', unit: 'm', step: 0.01, min: 0.01, max: 1, default: 0.15 },
  { id: 'slingLength', physicsKey: 'LS', label: 'Sling Length', unit: 'm', step: 0.05, min: 0.05, max: 1, default: 0.2 },
  { id: 'baseHeight', physicsKey: 'h', label: 'Base Height (CW Start)', unit: 'm', step: 0.05, min: 0.1, max: 2, default: 0.4 },
  { id: 'pinHeight', physicsKey: 'pinHeight', label: 'Fixed Pin Height', unit: 'm', step: 0.05, min: 0.05, max: 1, default: 0.2 },
  { id: 'pinDistance', physicsKey: 'pinDistance', label: 'Pin Horizontal Distance', unit: 'm', step: 0.05, min: 0.05, max: 1, default: 0.2 },
  { id: 'counterweightMass', physicsKey: 'mW', label: 'Counterweight Mass', unit: 'kg', step: 1, min: 1, max: 100, default: 10 },
  { id: 'projectileMass', physicsKey: 'mP', label: 'Projectile Mass', unit: 'kg', step: 0.01, min: 0.01, max: 5, default: 0.1 },
  { id: 'releaseAngle', physicsKey: 'releaseAngle', label: 'Hook Angle', unit: '°', step: 5, min: -90, max: 0, default: -65 },
  { id: 'rampAngle', physicsKey: 'rampAngle', label: 'Ramp Angle', unit: '°', step: 1, min: 0, max: 45, default: 10 },
];

// ============================================================================
// Export
// ============================================================================

export const floatingArm: TrebuchetDesign = {
  id: 'floating',
  name: 'Floating Arm',
  parameterConfig,
  simulate: fatSimulate,
  createInitialSample: fatCreateInitialSample,
  computeGeometry: fatComputeGeometry,
};
