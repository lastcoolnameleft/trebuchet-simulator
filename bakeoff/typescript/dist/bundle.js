"use strict";
(() => {
  // src/geometry.ts
  function computeTrebuchetGeometry(params, pose) {
    const { Aq, Wq, Sq } = pose;
    const { LAl, LAs, LAcg, LW, LS } = params;
    return {
      armCg: {
        x: LAcg * Math.sin(Aq),
        y: -LAcg * Math.cos(Aq)
      },
      counterweightAttach: {
        x: LAs * Math.sin(Aq),
        y: -LAs * Math.cos(Aq)
      },
      counterweight: {
        x: LAs * Math.sin(Aq) + LW * Math.sin(Aq + Wq),
        y: -LAs * Math.cos(Aq) - LW * Math.cos(Aq + Wq)
      },
      slingAttach: {
        x: -LAl * Math.sin(Aq),
        y: LAl * Math.cos(Aq)
      },
      projectile: {
        x: -LAl * Math.sin(Aq) - LS * Math.sin(Aq + Sq),
        y: LAl * Math.cos(Aq) + LS * Math.cos(Aq + Sq)
      }
    };
  }
  function computeProjectileVelocity(params, pose, angularVelocity) {
    const theta = pose.Aq + pose.Sq;
    const thetaDot = angularVelocity.Aw + angularVelocity.Sw;
    return {
      x: -params.LAl * Math.cos(pose.Aq) * angularVelocity.Aw - params.LS * Math.cos(theta) * thetaDot,
      y: -params.LAl * Math.sin(pose.Aq) * angularVelocity.Aw - params.LS * Math.sin(theta) * thetaDot
    };
  }
  function computeProjectileAcceleration(params, pose, angularVelocity, angularAcceleration) {
    const theta = pose.Aq + pose.Sq;
    const thetaDot = angularVelocity.Aw + angularVelocity.Sw;
    const thetaAcc = angularAcceleration.Aacc + angularAcceleration.Sacc;
    return {
      x: params.LAl * Math.sin(pose.Aq) * angularVelocity.Aw ** 2 - params.LAl * Math.cos(pose.Aq) * angularAcceleration.Aacc + params.LS * Math.sin(theta) * thetaDot ** 2 - params.LS * Math.cos(theta) * thetaAcc,
      y: -params.LAl * Math.cos(pose.Aq) * angularVelocity.Aw ** 2 - params.LAl * Math.sin(pose.Aq) * angularAcceleration.Aacc - params.LS * Math.cos(theta) * thetaDot ** 2 - params.LS * Math.sin(theta) * thetaAcc
    };
  }
  function computeCounterweightVelocity(params, pose, angularVelocity) {
    const theta = pose.Aq + pose.Wq;
    const thetaDot = angularVelocity.Aw + angularVelocity.Ww;
    return {
      x: params.LAs * Math.cos(pose.Aq) * angularVelocity.Aw + params.LW * Math.cos(theta) * thetaDot,
      y: params.LAs * Math.sin(pose.Aq) * angularVelocity.Aw + params.LW * Math.sin(theta) * thetaDot
    };
  }
  function computeArmCgVelocity(params, pose, angularVelocity) {
    return {
      x: params.LAcg * Math.cos(pose.Aq) * angularVelocity.Aw,
      y: params.LAcg * Math.sin(pose.Aq) * angularVelocity.Aw
    };
  }

  // src/physics.ts
  var DEG_TO_RAD = Math.PI / 180;
  var RAD_TO_DEG = 180 / Math.PI;
  var FIXED_DT = 1e-3;
  var MAX_TIME = 12;
  var EPSILON = 1e-6;
  var defaultParams = () => {
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
      IA3: mA * (LAl + LAs) ** 2 / 12,
      IW3: mW * LW ** 2 / 12,
      releaseAngle: 45,
      Grav: 9.81,
      startArmAngleDeg: 45
    };
  };
  function normalizeParams(overrides = {}) {
    const base = defaultParams();
    const merged = { ...base, ...overrides };
    return {
      ...merged,
      LAcg: overrides.LAcg ?? (merged.LAs - merged.LAl) / 2,
      IA3: overrides.IA3 ?? merged.mA * (merged.LAl + merged.LAs) ** 2 / 12,
      IW3: overrides.IW3 ?? merged.mW * merged.LW ** 2 / 12
    };
  }
  function createInitialSample(input = {}) {
    const params = normalizeParams(input);
    const Aq = params.startArmAngleDeg * DEG_TO_RAD;
    const Wq = 0;
    const Sq = solveGroundSlingAngle(params, Aq);
    const sample = makeSample(params, 0, "ground", {
      Aq,
      Wq,
      Sq,
      Aw: 0,
      Ww: 0,
      Sw: 0
    });
    return { params, sample };
  }
  function simulateTrebuchet(input = {}) {
    const params = normalizeParams(input);
    const samples = [];
    const initialAq = params.startArmAngleDeg * DEG_TO_RAD;
    let Aq = initialAq;
    let Wq = 0;
    let Sq = solveGroundSlingAngle(params, Aq);
    let Aw = 0;
    let Ww = 0;
    let Sw = groundSlingRate(params, Aq, Sq, Aw);
    let time = 0;
    let stage = "ground";
    let liftOffTime = 0;
    let releaseTime = 0;
    let releasedSlingAngle = Sq;
    let releaseSpeed = 0;
    let releaseHeight = params.h - computeTrebuchetGeometry(params, { Aq, Wq, Sq }).projectile.y;
    let peakSpeed = 0;
    let maxHeight = 0;
    let lastReleaseDelta = null;
    let flight = null;
    const pushSample = (sample) => {
      samples.push(sample);
      peakSpeed = Math.max(peakSpeed, sample.projectileSpeed);
      maxHeight = Math.max(maxHeight, params.h - sample.projectileY);
    };
    pushSample(makeSample(params, time, stage, { Aq, Wq, Sq, Aw, Ww, Sw }));
    while (time < MAX_TIME) {
      if (stage === "ground") {
        const next2 = rk4([Aq, Wq, Aw, Ww], FIXED_DT, (state) => groundDerivative(params, state));
        Aq = next2[0];
        Wq = next2[1];
        Aw = next2[2];
        Ww = next2[3];
        Sq = solveGroundSlingAngle(params, Aq);
        Sw = groundSlingRate(params, Aq, Sq, Aw);
        time += FIXED_DT;
        const lifted = liftedAcceleration(params, { Aq, Wq, Sq, Aw, Ww, Sw });
        const projAcc = computeProjectileAcceleration(
          params,
          { Aq, Sq },
          { Aw, Sw },
          { Aacc: lifted.Aacc, Sacc: lifted.Sacc }
        );
        if (projAcc.y < 0) {
          stage = "lifted";
          liftOffTime = time;
        }
        pushSample(makeSample(params, time, stage, { Aq, Wq, Sq, Aw, Ww, Sw }));
        continue;
      }
      if (stage === "lifted") {
        const next2 = rk4([Aq, Wq, Sq, Aw, Ww, Sw], FIXED_DT, (state) => liftedDerivative(params, state));
        Aq = next2[0];
        Wq = next2[1];
        Sq = next2[2];
        Aw = next2[3];
        Ww = next2[4];
        Sw = next2[5];
        time += FIXED_DT;
        const currentSample = makeSample(params, time, stage, { Aq, Wq, Sq, Aw, Ww, Sw });
        const delta = currentSample.releaseAngleNow - params.releaseAngle;
        const readyToRelease = currentSample.projectileVx > 0;
        if (readyToRelease && lastReleaseDelta !== null && lastReleaseDelta > 0 && delta <= 0) {
          stage = "flight";
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
            Ww
          };
          pushSample({ ...currentSample, stage: "flight" });
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
        (state) => freeFlightDerivative(params, state)
      );
      flight = {
        x: next[0],
        y: next[1],
        vx: next[2],
        vy: next[3],
        Aq: next[4],
        Wq: next[5],
        Aw: next[6],
        Ww: next[7]
      };
      time += FIXED_DT;
      const sample = makeSample(
        params,
        time,
        "flight",
        {
          Aq: flight.Aq,
          Wq: flight.Wq,
          Sq: releasedSlingAngle,
          Aw: flight.Aw,
          Ww: flight.Ww,
          Sw: 0
        },
        {
          x: flight.x,
          y: flight.y,
          vx: flight.vx,
          vy: flight.vy
        }
      );
      pushSample(sample);
      if (flight.y > params.h) {
        stage = "done";
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
        liftOffTime
      }
    };
  }
  function findSampleAtTime(samples, time) {
    if (samples.length === 0) {
      throw new Error("No simulation samples available.");
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
      releaseAngleNow: lerp(prev.releaseAngleNow, next.releaseAngleNow, alpha)
    };
  }
  function makeSample(params, time, stage, liftedState, projectileOverride) {
    const geometry = computeTrebuchetGeometry(params, liftedState);
    const projectileVelocity = projectileOverride ? { x: projectileOverride.vx, y: projectileOverride.vy } : computeProjectileVelocity(params, liftedState, liftedState);
    const projectile = projectileOverride ? { x: projectileOverride.x, y: projectileOverride.y } : geometry.projectile;
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
      releaseAngleNow: Math.atan2(-projectileVelocity.y, projectileVelocity.x) * RAD_TO_DEG
    };
  }
  function solveGroundSlingAngle(params, Aq) {
    const cosTerm = clamp((params.h - params.LAl * Math.cos(Aq)) / params.LS, -1, 1);
    return Math.acos(cosTerm) - Aq;
  }
  function groundSlingRate(params, Aq, Sq, Aw) {
    const denominator = params.LS * Math.sin(Aq + Sq);
    if (Math.abs(denominator) < EPSILON) {
      return 0;
    }
    return -((params.LAl * Math.sin(Aq) + params.LS * Math.sin(Aq + Sq)) * Aw) / denominator;
  }
  function groundDerivative(params, state) {
    const [Aq, Wq, Aw, Ww] = state;
    const system = createGroundSystem(params);
    const [Aacc, Wacc] = reducedAcceleration([Aq, Wq], [Aw, Ww], system);
    return [Aw, Ww, Aacc, Wacc];
  }
  function createGroundSystem(params) {
    return {
      kinetic: ([Aq, Wq], [Aw, Ww]) => {
        const Sq = solveGroundSlingAngle(params, Aq);
        const Sw = groundSlingRate(params, Aq, Sq, Aw);
        const pose = { Aq, Wq, Sq };
        const armVelocity = computeArmCgVelocity(params, pose, { Aw });
        const weightVelocity = computeCounterweightVelocity(params, pose, { Aw, Ww });
        const projectileVelocity = computeProjectileVelocity(params, pose, { Aw, Sw });
        return 0.5 * params.mA * (armVelocity.x ** 2 + armVelocity.y ** 2) + 0.5 * params.IA3 * Aw ** 2 + 0.5 * params.mW * (weightVelocity.x ** 2 + weightVelocity.y ** 2) + 0.5 * params.IW3 * (Aw + Ww) ** 2 + 0.5 * params.mP * (projectileVelocity.x ** 2 + projectileVelocity.y ** 2);
      },
      potential: ([Aq, Wq]) => {
        const Sq = solveGroundSlingAngle(params, Aq);
        const geometry = computeTrebuchetGeometry(params, { Aq, Wq, Sq });
        return -params.Grav * (params.mA * geometry.armCg.y + params.mW * geometry.counterweight.y + params.mP * geometry.projectile.y);
      }
    };
  }
  function createArmOnlySystem(params) {
    return {
      kinetic: ([Aq, Wq], [Aw, Ww]) => {
        const pose = { Aq, Wq, Sq: 0 };
        const armVelocity = computeArmCgVelocity(params, pose, { Aw });
        const weightVelocity = computeCounterweightVelocity(params, pose, { Aw, Ww });
        return 0.5 * params.mA * (armVelocity.x ** 2 + armVelocity.y ** 2) + 0.5 * params.IA3 * Aw ** 2 + 0.5 * params.mW * (weightVelocity.x ** 2 + weightVelocity.y ** 2) + 0.5 * params.IW3 * (Aw + Ww) ** 2;
      },
      potential: ([Aq, Wq]) => {
        const geometry = computeTrebuchetGeometry(params, { Aq, Wq, Sq: 0 });
        return -params.Grav * (params.mA * geometry.armCg.y + params.mW * geometry.counterweight.y);
      }
    };
  }
  function reducedAcceleration(q, qd, system) {
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
  function buildMassMatrix(dof, q, kinetic) {
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
  function differentiateMassMatrix(dof, q, axis, kinetic) {
    const plus = q.slice();
    const minus = q.slice();
    plus[axis] += EPSILON;
    minus[axis] -= EPSILON;
    const mPlus = buildMassMatrix(dof, plus, kinetic);
    const mMinus = buildMassMatrix(dof, minus, kinetic);
    return mPlus.map((row, rowIndex) => row.map((value, colIndex) => (value - mMinus[rowIndex][colIndex]) / (2 * EPSILON)));
  }
  function liftedDerivative(params, state) {
    const [Aq, Wq, Sq, Aw, Ww, Sw] = state;
    const accel = liftedAcceleration(params, { Aq, Wq, Sq, Aw, Ww, Sw });
    return [Aw, Ww, Sw, accel.Aacc, accel.Wacc, accel.Sacc];
  }
  function liftedAcceleration(params, state) {
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
      mW
    } = params;
    const M11 = IA3 + IW3 + mA * LAcg ** 2 + mP * (LAl ** 2 + LS ** 2 + 2 * LAl * LS * Math.cos(Sq)) + mW * (LAs ** 2 + LW ** 2 + 2 * LAs * LW * Math.cos(Wq));
    const M12 = IW3 + LW * mW * (LW + LAs * Math.cos(Wq));
    const M13 = LS * mP * (LS + LAl * Math.cos(Sq));
    const M22 = IW3 + mW * LW ** 2;
    const M33 = mP * LS ** 2;
    const r1 = Grav * LAcg * mA * Math.sin(Aq) + Grav * mP * (LAl * Math.sin(Aq) + LS * Math.sin(Aq + Sq)) - Grav * mW * (LAs * Math.sin(Aq) + LW * Math.sin(Aq + Wq)) - LAl * LS * mP * Math.sin(Sq) * (Aw ** 2 - (Aw + Sw) ** 2) - LAs * LW * mW * Math.sin(Wq) * (Aw ** 2 - (Aw + Ww) ** 2);
    const r2 = -LW * mW * (Grav * Math.sin(Aq + Wq) + LAs * Math.sin(Wq) * Aw ** 2);
    const r3 = LS * mP * (Grav * Math.sin(Aq + Sq) - LAl * Math.sin(Sq) * Aw ** 2);
    const [Aacc, Wacc, Sacc] = solveLinearSystem(
      [
        [M11, M12, M13],
        [M12, M22, 0],
        [M13, 0, M33]
      ],
      [r1, r2, r3]
    );
    return { Aacc, Wacc, Sacc };
  }
  function freeFlightDerivative(params, state) {
    const [x, y, vx, vy, Aq, Wq, Aw, Ww] = state;
    const [Aacc, Wacc] = reducedAcceleration([Aq, Wq], [Aw, Ww], createArmOnlySystem(params));
    return [vx, vy, 0, params.Grav, Aw, Ww, Aacc, Wacc].map((value, index) => index < 2 ? value : value);
  }
  function rk4(state, dt, derivative) {
    const k1 = derivative(state);
    const k2 = derivative(addScaled(state, k1, dt / 2));
    const k3 = derivative(addScaled(state, k2, dt / 2));
    const k4 = derivative(addScaled(state, k3, dt));
    return state.map((value, index) => value + dt / 6 * (k1[index] + 2 * k2[index] + 2 * k3[index] + k4[index]));
  }
  function addScaled(state, derivative, scale) {
    return state.map((value, index) => value + derivative[index] * scale);
  }
  function centralDifference(fn, q, axis) {
    const plus = q.slice();
    const minus = q.slice();
    plus[axis] += EPSILON;
    minus[axis] -= EPSILON;
    return (fn(plus) - fn(minus)) / (2 * EPSILON);
  }
  function solveLinearSystem(matrix, rhs) {
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
        throw new Error("Singular mass matrix encountered.");
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
  function unitVector(length, activeIndex) {
    return Array.from({ length }, (_, index) => index === activeIndex ? 1 : 0);
  }
  function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }
  function lerp(a, b, alpha) {
    return a + (b - a) * alpha;
  }

  // src/renderer.ts
  var TrebuchetRenderer = class {
    constructor(canvas2) {
      this.canvas = canvas2;
      this.viewport = { scale: 40, originX: 180, originY: 160 };
      this.currentResult = null;
      const context = canvas2.getContext("2d");
      if (!context) {
        throw new Error("Unable to create canvas context.");
      }
      this.ctx = context;
    }
    setSimulation(result) {
      this.currentResult = result;
      if (result) {
        this.viewport = computeViewport(this.canvas, result);
      }
    }
    drawPreview(params, sample) {
      this.currentResult = null;
      this.viewport = computeViewport(this.canvas, {
        params,
        samples: [sample],
        stats: {
          range: sample.projectileX,
          maxHeight: params.h - sample.projectileY,
          peakSpeed: 0,
          releaseSpeed: 0,
          releaseHeight: 0,
          releaseTime: 0,
          flightTime: 0,
          totalTime: 0,
          liftOffTime: 0
        }
      });
      this.drawScene(params, sample, [sample]);
    }
    render(result, time) {
      this.currentResult = result;
      const sample = findSampleAtTime(result.samples, time);
      this.drawScene(result.params, sample, result.samples);
    }
    drawScene(params, sample, trailSource) {
      const { ctx, canvas: canvas2 } = this;
      const geometry = computeTrebuchetGeometry(params, sample);
      const pivot = worldToScreen(this.viewport, 0, 0);
      const groundY = worldToScreen(this.viewport, 0, params.h).y;
      ctx.clearRect(0, 0, canvas2.width, canvas2.height);
      drawBackdrop(ctx, canvas2, groundY);
      drawGround(ctx, canvas2, groundY);
      ctx.save();
      ctx.strokeStyle = "#64748b";
      ctx.lineWidth = 10;
      ctx.lineCap = "round";
      ctx.beginPath();
      ctx.moveTo(pivot.x, groundY);
      ctx.lineTo(pivot.x, pivot.y);
      ctx.stroke();
      ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.moveTo(pivot.x - 24, groundY);
      ctx.lineTo(pivot.x + 24, groundY);
      ctx.stroke();
      ctx.restore();
      const armStart = worldToScreen(this.viewport, geometry.counterweightAttach.x, geometry.counterweightAttach.y);
      const armEnd = worldToScreen(this.viewport, geometry.slingAttach.x, geometry.slingAttach.y);
      const weight = worldToScreen(this.viewport, geometry.counterweight.x, geometry.counterweight.y);
      const projectile = worldToScreen(this.viewport, sample.projectileX, sample.projectileY);
      drawTrail(ctx, this.viewport, trailSource, sample.time);
      ctx.save();
      ctx.strokeStyle = "#e2e8f0";
      ctx.lineWidth = Math.max(4, this.viewport.scale * 0.12);
      ctx.beginPath();
      ctx.moveTo(armStart.x, armStart.y);
      ctx.lineTo(armEnd.x, armEnd.y);
      ctx.stroke();
      ctx.strokeStyle = "#cbd5e1";
      ctx.lineWidth = Math.max(2, this.viewport.scale * 0.07);
      ctx.beginPath();
      ctx.moveTo(armStart.x, armStart.y);
      ctx.lineTo(weight.x, weight.y);
      ctx.stroke();
      const slingTip = sample.stage === "flight" ? worldToScreen(
        this.viewport,
        geometry.slingAttach.x - params.LS * Math.sin(sample.Aq + sample.Sq),
        geometry.slingAttach.y + params.LS * Math.cos(sample.Aq + sample.Sq)
      ) : projectile;
      ctx.strokeStyle = "#f8fafc";
      ctx.beginPath();
      ctx.moveTo(armEnd.x, armEnd.y);
      ctx.lineTo(slingTip.x, slingTip.y);
      ctx.stroke();
      ctx.restore();
      drawPivot(ctx, pivot);
      drawCounterweight(ctx, weight, this.viewport.scale);
      drawProjectile(ctx, projectile, this.viewport.scale);
      drawHud(ctx, sample, params, canvas2);
    }
  };
  function computeViewport(canvas2, result) {
    let minX = -result.params.LAl - result.params.LS - 2;
    let maxX = result.params.LAs + result.params.LW + 2;
    let minY = -4;
    let maxY = result.params.h + 2;
    for (let index = 0; index < result.samples.length; index += Math.max(1, Math.floor(result.samples.length / 300))) {
      const sample = result.samples[index];
      const geometry = computeTrebuchetGeometry(result.params, sample);
      const points = [
        geometry.counterweight,
        geometry.counterweightAttach,
        geometry.slingAttach,
        { x: sample.projectileX, y: sample.projectileY }
      ];
      for (const point of points) {
        minX = Math.min(minX, point.x);
        maxX = Math.max(maxX, point.x);
        minY = Math.min(minY, point.y);
        maxY = Math.max(maxY, point.y);
      }
    }
    maxY = Math.max(maxY, result.params.h + 1.5);
    const width = Math.max(8, maxX - minX);
    const height = Math.max(8, maxY - minY);
    const scale = Math.min((canvas2.width - 120) / width, (canvas2.height - 120) / height);
    return {
      scale,
      originX: 60 - minX * scale,
      originY: 60 - minY * scale
    };
  }
  function worldToScreen(viewport, x, y) {
    return {
      x: viewport.originX + x * viewport.scale,
      y: viewport.originY + y * viewport.scale
    };
  }
  function drawBackdrop(ctx, canvas2, groundY) {
    const sky = ctx.createLinearGradient(0, 0, 0, groundY);
    sky.addColorStop(0, "#0f172a");
    sky.addColorStop(1, "#2563eb");
    ctx.fillStyle = sky;
    ctx.fillRect(0, 0, canvas2.width, groundY);
    const field = ctx.createLinearGradient(0, groundY, 0, canvas2.height);
    field.addColorStop(0, "#166534");
    field.addColorStop(1, "#14532d");
    ctx.fillStyle = field;
    ctx.fillRect(0, groundY, canvas2.width, canvas2.height - groundY);
  }
  function drawGround(ctx, canvas2, groundY) {
    ctx.save();
    ctx.strokeStyle = "#bbf7d0";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(0, groundY);
    ctx.lineTo(canvas2.width, groundY);
    ctx.stroke();
    ctx.restore();
  }
  function drawPivot(ctx, pivot) {
    ctx.save();
    ctx.fillStyle = "#f8fafc";
    ctx.beginPath();
    ctx.arc(pivot.x, pivot.y, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }
  function drawCounterweight(ctx, point, scale) {
    const size = Math.max(14, scale * 0.24);
    ctx.save();
    ctx.fillStyle = "#f59e0b";
    ctx.fillRect(point.x - size / 2, point.y - size / 2, size, size);
    ctx.restore();
  }
  function drawProjectile(ctx, point, scale) {
    ctx.save();
    ctx.fillStyle = "#fb7185";
    ctx.beginPath();
    ctx.arc(point.x, point.y, Math.max(6, scale * 0.09), 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }
  function drawTrail(ctx, viewport, samples, time) {
    const trail = samples.filter((sample) => sample.stage === "flight" && sample.time <= time).slice(-160);
    if (trail.length < 2) {
      return;
    }
    ctx.save();
    ctx.strokeStyle = "rgba(251, 113, 133, 0.45)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    trail.forEach((sample, index) => {
      const point = worldToScreen(viewport, sample.projectileX, sample.projectileY);
      if (index === 0) {
        ctx.moveTo(point.x, point.y);
      } else {
        ctx.lineTo(point.x, point.y);
      }
    });
    ctx.stroke();
    ctx.restore();
  }
  function drawHud(ctx, sample, params, canvas2) {
    const stageLabel = sample.stage === "ground" ? "Ground constrained" : sample.stage === "lifted" ? "Projectile lifted" : "Free flight";
    ctx.save();
    ctx.fillStyle = "rgba(15, 23, 42, 0.78)";
    ctx.fillRect(canvas2.width - 250, 18, 220, 110);
    ctx.fillStyle = "#f8fafc";
    ctx.font = "600 16px Inter, system-ui, sans-serif";
    ctx.fillText(stageLabel, canvas2.width - 232, 42);
    ctx.font = "14px Inter, system-ui, sans-serif";
    ctx.fillStyle = "#cbd5e1";
    ctx.fillText(`t = ${sample.time.toFixed(3)} s`, canvas2.width - 232, 66);
    ctx.fillText(`speed = ${sample.projectileSpeed.toFixed(2)} m/s`, canvas2.width - 232, 88);
    ctx.fillText(`flight angle = ${sample.releaseAngleNow.toFixed(1)}\xB0`, canvas2.width - 232, 110);
    ctx.restore();
    void params;
  }

  // src/app.ts
  var fields = [
    { key: "LAl", label: "Arm long side [m]", step: "0.1" },
    { key: "LAs", label: "Arm short side [m]", step: "0.1" },
    { key: "LAcg", label: "Arm CG distance [m]", step: "0.1" },
    { key: "LW", label: "CW hanging length [m]", step: "0.1" },
    { key: "LS", label: "Sling length [m]", step: "0.1" },
    { key: "h", label: "Pivot height [m]", step: "0.1" },
    { key: "mA", label: "Arm mass [kg]", step: "1" },
    { key: "mW", label: "Counterweight mass [kg]", step: "1" },
    { key: "mP", label: "Projectile mass [kg]", step: "0.1" },
    { key: "IA3", label: "Arm inertia [kg\xB7m\xB2]", step: "0.1" },
    { key: "IW3", label: "CW inertia [kg\xB7m\xB2]", step: "0.1" },
    { key: "releaseAngle", label: "Release angle [deg]", step: "0.1" },
    { key: "startArmAngleDeg", label: "Start arm angle [deg]", step: "0.1" },
    { key: "Grav", label: "Gravity [m/s\xB2]", step: "0.01" }
  ];
  var form = document.getElementById("controls");
  var fireButton = document.getElementById("fireButton");
  var resetButton = document.getElementById("resetButton");
  var statsRoot = document.getElementById("stats");
  var status = document.getElementById("status");
  var canvas = document.getElementById("simCanvas");
  var renderer = new TrebuchetRenderer(canvas);
  var defaults = defaultParams();
  var currentResult = null;
  var animationFrame = 0;
  var animationStart = 0;
  buildForm(defaults);
  renderPreview();
  status.innerHTML = "<strong>Ready.</strong> Adjust parameters and press Fire.";
  window.addEventListener("resize", () => {
    if (currentResult) {
      renderer.setSimulation(currentResult);
      renderer.render(currentResult, 0);
      return;
    }
    renderPreview();
  });
  form.addEventListener("input", () => {
    if (!currentResult) {
      renderPreview();
    }
  });
  fireButton.addEventListener("click", () => {
    cancelAnimationFrame(animationFrame);
    try {
      const params = readParams();
      currentResult = simulateTrebuchet(params);
      renderer.setSimulation(currentResult);
      updateStats(currentResult);
      animationStart = performance.now();
      status.innerHTML = "<strong>Simulating.</strong> Analytical RK4 trajectory is playing back at 1\xD7 speed.";
      tick();
    } catch (error) {
      const message = error instanceof Error ? error.message : "Unknown simulation error.";
      status.innerHTML = `<strong>Simulation error.</strong> ${message}`;
      currentResult = null;
      renderPreview();
    }
  });
  resetButton.addEventListener("click", () => {
    cancelAnimationFrame(animationFrame);
    currentResult = null;
    updateStats(null);
    renderPreview();
    status.innerHTML = "<strong>Reset.</strong> Trebuchet returned to its pre-fire pose.";
  });
  function tick() {
    if (!currentResult) {
      return;
    }
    const elapsed = (performance.now() - animationStart) / 1e3;
    const displayTime = Math.min(elapsed, currentResult.stats.totalTime);
    renderer.render(currentResult, displayTime);
    if (displayTime < currentResult.stats.totalTime) {
      animationFrame = requestAnimationFrame(tick);
      return;
    }
    status.innerHTML = `<strong>Impact.</strong> Range ${currentResult.stats.range.toFixed(2)} m after ${currentResult.stats.totalTime.toFixed(2)} s.`;
  }
  function renderPreview() {
    const { params, sample } = createInitialSample(readParamsSafe());
    renderer.drawPreview(params, sample);
  }
  function buildForm(params) {
    form.innerHTML = "";
    fields.forEach(({ key, label, step }) => {
      const wrapper = document.createElement("label");
      wrapper.innerHTML = `${label}<input name="${key}" type="number" step="${step ?? "any"}" value="${params[key]}" />`;
      form.appendChild(wrapper);
    });
  }
  function readParams() {
    const values = {};
    new FormData(form).forEach((value, key) => {
      values[key] = Number(value);
    });
    for (const [key, value] of Object.entries(values)) {
      if (!Number.isFinite(value)) {
        throw new Error(`Invalid numeric value for ${key}.`);
      }
    }
    return values;
  }
  function readParamsSafe() {
    try {
      return readParams();
    } catch {
      return defaults;
    }
  }
  function updateStats(result) {
    const cards = result ? [
      ["Range", `${result.stats.range.toFixed(2)} m`],
      ["Max height", `${result.stats.maxHeight.toFixed(2)} m`],
      ["Peak speed", `${result.stats.peakSpeed.toFixed(2)} m/s`],
      ["Release speed", `${result.stats.releaseSpeed.toFixed(2)} m/s`],
      ["Lift-off", `${result.stats.liftOffTime.toFixed(3)} s`],
      ["Flight time", `${result.stats.flightTime.toFixed(3)} s`],
      ["Release height", `${result.stats.releaseHeight.toFixed(2)} m`],
      ["Total time", `${result.stats.totalTime.toFixed(3)} s`]
    ] : [
      ["Range", "\u2014"],
      ["Max height", "\u2014"],
      ["Peak speed", "\u2014"],
      ["Release speed", "\u2014"],
      ["Lift-off", "\u2014"],
      ["Flight time", "\u2014"],
      ["Release height", "\u2014"],
      ["Total time", "\u2014"]
    ];
    statsRoot.innerHTML = cards.map(
      ([label, value]) => `<div class="stat"><div class="label">${label}</div><div class="value">${value}</div></div>`
    ).join("");
  }
  updateStats(null);
})();
