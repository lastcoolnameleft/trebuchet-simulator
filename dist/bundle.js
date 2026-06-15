"use strict";
(() => {
  // src/geometry.ts
  function computeTrebuchetGeometry(params, pose) {
    const { Aq, Wq, Sq } = pose;
    const { LAl, LAs, LAcg, LW, LS } = params;
    return {
      armCg: {
        x: -LAcg * Math.sin(Aq),
        y: -LAcg * Math.cos(Aq)
      },
      counterweightAttach: {
        x: LAs * Math.sin(Aq),
        y: LAs * Math.cos(Aq)
      },
      counterweight: {
        x: LAs * Math.sin(Aq) + LW * Math.sin(Aq + Wq),
        y: LAs * Math.cos(Aq) + LW * Math.cos(Aq + Wq)
      },
      slingAttach: {
        x: -LAl * Math.sin(Aq),
        y: -LAl * Math.cos(Aq)
      },
      projectile: {
        x: -LAl * Math.sin(Aq) - LS * Math.sin(Aq + Sq),
        y: -(LAl * Math.cos(Aq) + LS * Math.cos(Aq + Sq))
      }
    };
  }

  // src/physics.ts
  var DEG_TO_RAD = Math.PI / 180;
  var RAD_TO_DEG = 180 / Math.PI;
  var FIXED_DT = 1e-3;
  var MAX_TIME = 6;
  var MAX_FLIGHT_TIME = 20;
  var EPS = 1e-9;
  var EVENT_EPS = 1e-7;
  var AIR_DENSITY = 1.225;
  var DRAG_COEFFICIENT = 0.47;
  var PROJECTILE_DIAMETER = 0.0759;
  var WIND_SPEED = 0;
  var ENABLE_AIR_DRAG = true;
  var defaultParams = () => ({
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
    startArmAngleDeg: 0
    // 0 = auto-compute from geometry
  });
  function normalizeParams(input = {}) {
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
      windSpeed: WIND_SPEED
    };
  }
  function createInitialSample(input = {}) {
    const params = normalizeParams(input);
    validateGeometry(params);
    const Aq = computeInitialArmAngle(params);
    const Wq = -Aq;
    const Sq = groundSlingAngle(Aq, params);
    const sample = makeAttachedSample(params, 0, "ground", [Aq, Wq, Sq, 0, 0, 0]);
    return { params, sample };
  }
  function simulateTrebuchet(input = {}) {
    const params = normalizeParams(input);
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
        liftOffTime: bundle.tLiftoff
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
  function solveBundle(params) {
    validateGeometry(params);
    const Aq0 = computeInitialArmAngle(params);
    const Wq0 = -Aq0;
    const Sq0 = groundSlingAngle(Aq0, params);
    const y0Stage1 = [Aq0, Wq0, Sq0, 0, 0, 0];
    const initialMetric = liftOffMetric(y0Stage1, params);
    let stage1 = [{ time: 0, state: y0Stage1.slice() }];
    let tLiftoff = 0;
    let liftoffState = y0Stage1.slice();
    if (initialMetric < 0) {
      const stage1Run = integrateUntilEvent(
        y0Stage1,
        0,
        params.maxTime,
        params.maxStep,
        (state) => stage1Ode(state, params),
        (state) => liftOffEventMetric(state, params),
        1
      );
      if (!stage1Run.event) {
        throw new Error("Projectile never lifted off the ground within the simulation time.");
      }
      stage1 = stage1Run.trajectory;
      tLiftoff = stage1Run.event.time;
      const [Aq, Wq, _Sq, Aw, Ww] = stage1Run.event.state;
      const Sq = groundSlingAngle(Aq, params);
      const Sw = stage1SqDot(Aq, Sq, Aw, params);
      liftoffState = [Aq, Wq, Sq, Aw, Ww, Sw];
      replaceLast(stage1, { time: tLiftoff, state: liftoffState.slice() });
    }
    const stage2Run = integrateUntilEvent(
      liftoffState,
      tLiftoff,
      params.maxTime,
      params.maxStep,
      (state) => stage2Ode(state, params),
      (state) => releaseEvent(state, params),
      -1
    );
    if (!stage2Run.event) {
      throw new Error("Release angle was never reached. Try a lower release angle or different geometry.");
    }
    const stage2 = stage2Run.trajectory;
    const tRelease = stage2Run.event.time;
    const releaseState = stage2Run.event.state.slice();
    replaceLast(stage2, { time: tRelease, state: releaseState.slice() });
    const releaseKinematics = projectileKinematics(releaseState, params);
    const releasePos = { x: releaseKinematics.x, y: releaseKinematics.y };
    const releaseVel = { vx: releaseKinematics.vx, vy: releaseKinematics.vy };
    const y0Post = [releaseState[0], releaseState[1], releaseState[3], releaseState[4]];
    const y0Flight = [releaseKinematics.x, releaseKinematics.y, releaseKinematics.vx, releaseKinematics.vy];
    const tFlightEnd = tRelease + params.maxFlightTime;
    const postRelease = [{ time: tRelease, state: y0Post.slice() }];
    const flight = [{ time: tRelease, state: y0Flight.slice() }];
    let currentPost = y0Post.slice();
    let currentFlight = y0Flight.slice();
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
          -1
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
      throw new Error("Projectile did not land within the flight time limit.");
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
      initialProjectileX
    };
  }
  function buildSamples(bundle, params) {
    const samples = [];
    for (const point of bundle.stage1) {
      pushSample(samples, makeAttachedSample(params, point.time, "ground", point.state));
    }
    for (const point of bundle.stage2) {
      pushSample(samples, makeAttachedSample(params, point.time, "lifted", point.state));
    }
    const releaseSq = bundle.releaseState[2];
    const releaseMech = bundle.postRelease[0].state;
    for (let index = 0; index < bundle.flight.length; index += 1) {
      const point = bundle.flight[index];
      pushSample(samples, makeFlightSample(params, point.time, releaseMech, releaseSq, point.state));
    }
    return samples;
  }
  function pushSample(samples, sample) {
    const last = samples[samples.length - 1];
    if (last && Math.abs(last.time - sample.time) < EVENT_EPS) {
      samples[samples.length - 1] = sample;
      return;
    }
    samples.push(sample);
  }
  function makeAttachedSample(params, time, stage, state) {
    let normalizedState;
    if (stage === "ground") {
      const Sq = groundSlingAngle(state[0], params);
      normalizedState = [state[0], state[1], Sq, state[3], state[4], stage1SqDot(state[0], Sq, state[3], params)];
    } else {
      normalizedState = state.slice();
    }
    const { x, y, vx, vy } = projectileKinematics(normalizedState, params);
    return makeSample(time, stage, normalizedState, x, params.h - y, vx, -vy);
  }
  function makeFlightSample(params, time, mech, releaseSq, flightState) {
    const state = [mech[0], mech[1], releaseSq, mech[2], mech[3], 0];
    return makeSample(time, "flight", state, flightState[0], params.h - flightState[1], flightState[2], -flightState[3]);
  }
  function makeSample(time, stage, state, projectileX, projectileY, projectileVx, projectileVy) {
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
      releaseAngleNow: Math.atan2(-projectileVy, projectileVx) * RAD_TO_DEG
    };
  }
  function computeInitialArmAngle(params) {
    if (params.startArmAngleDeg !== 0) {
      return params.startArmAngleDeg * DEG_TO_RAD;
    }
    const ratio = params.h / params.LAl;
    if (Math.abs(ratio) <= 1) {
      return Math.PI - Math.acos(ratio);
    }
    return 14 * Math.PI / 15;
  }
  function validateGeometry(params) {
    const Aq0 = computeInitialArmAngle(params);
    const reach = Math.abs((-params.h - params.LAl * Math.cos(Aq0)) / params.LS);
    if (reach > 1) {
      throw new Error("Initial geometry cannot place the projectile on the ground. Increase sling length, lower the pivot, or reduce the initial arm angle.");
    }
  }
  function groundSlingAngle(Aq, params) {
    const cosTotal = clamp((-params.h - params.LAl * Math.cos(Aq)) / params.LS, -1, 1);
    const total = 2 * Math.PI - Math.acos(cosTotal);
    return total - Aq;
  }
  function stage1SqDot(Aq, Sq, Aw, params) {
    const sAs = Math.sin(Aq + Sq);
    return -safeDiv((params.LAl * Math.sin(Aq) + params.LS * sAs) * Aw, params.LS * sAs);
  }
  function stage1Components(Aq, Wq, Aw, Ww, params) {
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
    const M11 = -mP * LAl ** 2 * (-1 + safeDiv(2 * sA * cSq, sAs)) + IA3 + IW3 + mA * LAcg ** 2 + mP * LAl ** 2 * safeDiv(sA ** 2, sAs ** 2) + mW * (LAs ** 2 + LW ** 2 + 2 * LAs * LW * cW);
    const M12 = IW3 + LW * mW * (LW + LAs * cW);
    const M22 = IW3 + mW * LW ** 2;
    const r1 = Grav * LAcg * mA * sA + LAl * LS * mP * (sSq * (Aw + Sw) ** 2 + cSq * accelTerm) + LAl * mP * sA * safeDiv(LAl * sSq * Aw ** 2 - LS * accelTerm, sAs) - Grav * mW * (LAs * sA + LW * Math.sin(Aq + Wq)) - LAs * LW * mW * sW * (Aw ** 2 - (Aw + Ww) ** 2);
    const r2 = -LW * mW * (Grav * Math.sin(Aq + Wq) + LAs * sW * Aw ** 2);
    const det = M11 * M22 - M12 * M12;
    if (Math.abs(det) < EPS) {
      throw new Error("Stage 1 matrix became singular.");
    }
    const Awd = (r1 * M22 - r2 * M12) / det;
    const Wwd = -(r1 * M12 - r2 * M11) / det;
    const Swd = -safeDiv(cAs * Sw * (Sw + 2 * Aw), sAs) - (safeDiv(cAs, sAs) + safeDiv(LAl * cA, LS * sAs)) * Aw ** 2 - safeDiv((LAl * sA + LS * sAs) * Awd, LS * sAs);
    return { Sq, Sw, Awd, Wwd, Swd };
  }
  function stage1Ode(state, params) {
    const [Aq, Wq, _Sq, Aw, Ww] = state;
    const { Sq, Sw, Awd, Wwd, Swd } = stage1Components(Aq, Wq, Aw, Ww, params);
    return [Aw, Ww, Sw, Awd, Wwd, Swd];
  }
  function stage2Ode(state, params) {
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
    const r1 = Grav * LAcg * mA * Math.sin(Aq) + Grav * mP * (LAl * Math.sin(Aq) + LS * Math.sin(Aq + Sq)) - Grav * mW * (LAs * Math.sin(Aq) + LW * Math.sin(Aq + Wq)) - LAl * LS * mP * sSq * (Aw ** 2 - (Aw + Sw) ** 2) - LAs * LW * mW * sW * (Aw ** 2 - (Aw + Ww) ** 2);
    const r2 = -LW * mW * (Grav * Math.sin(Aq + Wq) + LAs * sW * Aw ** 2);
    const r3 = LS * mP * (Grav * Math.sin(Aq + Sq) - LAl * sSq * Aw ** 2);
    const [Awd, Wwd, Swd] = solve3x3(
      [
        [M11, M12, M13],
        [M12, M22, 0],
        [M13, 0, M33]
      ],
      [r1, r2, r3],
      "Stage 2 matrix became singular."
    );
    return [Aw, Ww, Sw, Awd, Wwd, Swd];
  }
  function postReleaseOde(state, params) {
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
      throw new Error("Post-release matrix became singular.");
    }
    const Awd = (r1 * M22 - r2 * M12) / det;
    const Wwd = -(r1 * M12 - r2 * M11) / det;
    return [Aw, Ww, Awd, Wwd];
  }
  function projectileKinematics(state, params) {
    const [Aq, _Wq, Sq, Aw, _Ww, Sw] = state;
    const x = -params.LAl * Math.sin(Aq) - params.LS * Math.sin(Aq + Sq);
    const y = params.h + params.LAl * Math.cos(Aq) + params.LS * Math.cos(Aq + Sq);
    const vx = -params.LAl * Math.cos(Aq) * Aw - params.LS * Math.cos(Aq + Sq) * (Aw + Sw);
    const vy = -params.LAl * Math.sin(Aq) * Aw - params.LS * Math.sin(Aq + Sq) * (Aw + Sw);
    return { x, y, vx, vy };
  }
  function liftOffMetric(state, params) {
    const deriv = stage2Ode(state, params);
    const Awd = deriv[3];
    const Swd = deriv[5];
    const [Aq, _Wq, Sq, Aw, _Ww, Sw] = state;
    const total = Aq + Sq;
    return params.LAl * Math.cos(Aq) * Aw ** 2 + params.LAl * Math.sin(Aq) * Awd + params.LS * Math.cos(total) * (Aw + Sw) ** 2 + params.LS * Math.sin(total) * (Awd + Swd);
  }
  function liftOffEventMetric(state, params) {
    const [Aq, Wq, _Sq, Aw, Ww] = state;
    const Sq = groundSlingAngle(Aq, params);
    const Sw = stage1SqDot(Aq, Sq, Aw, params);
    return liftOffMetric([Aq, Wq, Sq, Aw, Ww, Sw], params);
  }
  function releaseEvent(state, params) {
    const { vx, vy } = projectileKinematics(state, params);
    const speed = Math.hypot(vx, vy);
    if (vx <= 0 || speed < 0.5) {
      return -1;
    }
    return Math.atan2(vy, vx) - params.releaseAngleRad;
  }
  function flightOde(state, params) {
    const [x, y, vx, vy] = state;
    let dragTerm = 0;
    if (params.enableAirDrag) {
      const relX = vx - params.windSpeed;
      const speed = Math.hypot(relX, vy);
      dragTerm = params.airDensity * params.dragCoefficient * params.projectileArea * speed / (2 * params.mP);
    }
    const ax = -dragTerm * (vx - params.windSpeed);
    const ay = -params.Grav - dragTerm * vy;
    return [vx, vy, ax, ay];
  }
  function integrateUntilEvent(initialState, startTime, endTime, dt, derivative, metric, direction) {
    const trajectory = [{ time: startTime, state: initialState.slice() }];
    let currentState = initialState.slice();
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
  function crossesEvent(previous, next, direction) {
    if (direction > 0) {
      return previous < 0 && next >= 0;
    }
    return previous > 0 && next <= 0;
  }
  function refineEvent(leftTime, leftState, rightTime, rightState, metric, advance, direction) {
    const baseState = leftState.slice();
    let loTime = leftTime;
    let hiTime = rightTime;
    let loMetric = metric(leftState);
    let hiState = rightState.slice();
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
  function rk4Step(state, dt, derivative) {
    const k1 = derivative(state);
    const k2 = derivative(addScaled(state, k1, dt / 2));
    const k3 = derivative(addScaled(state, k2, dt / 2));
    const k4 = derivative(addScaled(state, k3, dt));
    return state.map((value, index) => value + dt / 6 * (k1[index] + 2 * k2[index] + 2 * k3[index] + k4[index]));
  }
  function addScaled(state, delta, scale) {
    return state.map((value, index) => value + delta[index] * scale);
  }
  function solve3x3(matrix, rhs, singularMessage) {
    const det = determinant3x3(matrix);
    if (Math.abs(det) < EPS) {
      throw new Error(singularMessage);
    }
    const det1 = determinant3x3([
      [rhs[0], matrix[0][1], matrix[0][2]],
      [rhs[1], matrix[1][1], matrix[1][2]],
      [rhs[2], matrix[2][1], matrix[2][2]]
    ]);
    const det2 = determinant3x3([
      [matrix[0][0], rhs[0], matrix[0][2]],
      [matrix[1][0], rhs[1], matrix[1][2]],
      [matrix[2][0], rhs[2], matrix[2][2]]
    ]);
    const det3 = determinant3x3([
      [matrix[0][0], matrix[0][1], rhs[0]],
      [matrix[1][0], matrix[1][1], rhs[1]],
      [matrix[2][0], matrix[2][1], rhs[2]]
    ]);
    return [det1 / det, det2 / det, det3 / det];
  }
  function determinant3x3(matrix) {
    return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
  }
  function replaceLast(items, value) {
    items[items.length - 1] = value;
  }
  function safeDiv(numerator, denominator) {
    if (Math.abs(denominator) < EPS) {
      denominator = denominator >= 0 ? EPS : -EPS;
    }
    return numerator / denominator;
  }
  function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }
  function lerp(a, b, alpha) {
    return a + (b - a) * alpha;
  }

  // src/renderer.ts
  var TrebuchetRenderer = class {
    constructor(canvas) {
      this.canvas = canvas;
      this.viewport = { scale: 40, originX: 180, originY: 160 };
      this.currentResult = null;
      const context = canvas.getContext("2d");
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
      const { ctx, canvas } = this;
      const geometry = computeTrebuchetGeometry(params, sample);
      const pivot = worldToScreen(this.viewport, 0, 0);
      const groundY = worldToScreen(this.viewport, 0, params.h).y;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      drawBackdrop(ctx, canvas, groundY);
      drawGround(ctx, canvas, groundY);
      ctx.save();
      ctx.strokeStyle = "#a87c4f";
      ctx.lineWidth = Math.max(8, this.viewport.scale * 0.2);
      ctx.lineCap = "round";
      ctx.beginPath();
      ctx.moveTo(pivot.x, groundY);
      ctx.lineTo(pivot.x, pivot.y);
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
        geometry.slingAttach.y - params.LS * Math.cos(sample.Aq + sample.Sq)
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
      drawHud(ctx, sample, params, canvas);
    }
  };
  function computeViewport(canvas, result) {
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
    const scale = Math.min((canvas.width - 120) / width, (canvas.height - 120) / height);
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
  function drawBackdrop(ctx, canvas, groundY) {
    const sky = ctx.createLinearGradient(0, 0, 0, groundY);
    sky.addColorStop(0, "#0f172a");
    sky.addColorStop(1, "#2563eb");
    ctx.fillStyle = sky;
    ctx.fillRect(0, 0, canvas.width, groundY);
    const field = ctx.createLinearGradient(0, groundY, 0, canvas.height);
    field.addColorStop(0, "#166534");
    field.addColorStop(1, "#14532d");
    ctx.fillStyle = field;
    ctx.fillRect(0, groundY, canvas.width, canvas.height - groundY);
  }
  function drawGround(ctx, canvas, groundY) {
    ctx.save();
    ctx.strokeStyle = "#bbf7d0";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(0, groundY);
    ctx.lineTo(canvas.width, groundY);
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
  function drawHud(ctx, sample, params, canvas) {
    const stageLabel = sample.stage === "ground" ? "Ground constrained" : sample.stage === "lifted" ? "Projectile lifted" : "Free flight";
    ctx.save();
    ctx.fillStyle = "rgba(15, 23, 42, 0.78)";
    ctx.fillRect(canvas.width - 250, 18, 220, 110);
    ctx.fillStyle = "#f8fafc";
    ctx.font = "600 16px Inter, system-ui, sans-serif";
    ctx.fillText(stageLabel, canvas.width - 232, 42);
    ctx.font = "14px Inter, system-ui, sans-serif";
    ctx.fillStyle = "#cbd5e1";
    ctx.fillText(`t = ${sample.time.toFixed(3)} s`, canvas.width - 232, 66);
    ctx.fillText(`speed = ${sample.projectileSpeed.toFixed(2)} m/s`, canvas.width - 232, 88);
    ctx.fillText(`flight angle = ${sample.releaseAngleNow.toFixed(1)}\xB0`, canvas.width - 232, 110);
    ctx.restore();
    void params;
  }

  // src/app.ts
  var parameterConfig = [
    { id: "projectileArmLength", physicsKey: "LAl", label: "Arm Length (Projectile)", unit: "m", step: 0.1, min: 0.5, max: 50, default: 2.07 },
    { id: "counterweightArmLength", physicsKey: "LAs", label: "Arm Length (Counterweight)", unit: "m", step: 0.1, min: 0.1, max: 20, default: 0.533 },
    { id: "armHeight", physicsKey: "h", label: "Pivot Height", unit: "m", step: 0.1, min: 0.5, max: 40, default: 1.524 },
    { id: "counterweightMass", physicsKey: "mW", label: "Counterweight Mass", unit: "kg", step: 1, min: 1, max: 5e3, default: 44.49 },
    { id: "cwHangLength", physicsKey: "LW", label: "CW Hanging Length", unit: "m", step: 0.1, min: 0.1, max: 10, default: 0.61 },
    { id: "projectileMass", physicsKey: "mP", label: "Projectile Mass", unit: "kg", step: 0.01, min: 0.01, max: 200, default: 0.149 },
    { id: "slingLength", physicsKey: "LS", label: "Sling Length", unit: "m", step: 0.1, min: 0.1, max: 30, default: 2.08 },
    { id: "armMass", physicsKey: "mA", label: "Arm Mass", unit: "kg", step: 0.5, min: 0.5, max: 500, default: 4.83 },
    { id: "releaseAngle", physicsKey: "releaseAngle", label: "Release Angle", unit: "\xB0", step: 1, min: 10, max: 80, default: 45 }
  ];
  var renderer;
  var currentResult = null;
  var animationFrame = 0;
  var animationStart = 0;
  var paused = true;
  var playSpeed = 1;
  var pausedTime = 0;
  document.addEventListener("DOMContentLoaded", () => {
    const canvas = document.getElementById("simulationCanvas");
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width;
    canvas.height = rect.height;
    renderer = new TrebuchetRenderer(canvas);
    buildParameterInputs();
    setupControls();
    renderPreview();
    document.getElementById("pauseBtn").textContent = "Fire";
  });
  window.addEventListener("resize", () => {
    const canvas = document.getElementById("simulationCanvas");
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
  function buildParameterInputs() {
    const container = document.querySelector(".section .parameters");
    if (!container) return;
    container.innerHTML = "";
    parameterConfig.forEach((param) => {
      const group = document.createElement("div");
      group.className = "param-group";
      const label = document.createElement("label");
      label.setAttribute("for", param.id);
      label.textContent = param.label;
      const input = document.createElement("input");
      input.type = "number";
      input.id = param.id;
      input.step = String(param.step);
      input.min = String(param.min);
      input.max = String(param.max);
      input.value = String(param.default);
      const unit = document.createElement("span");
      unit.className = "unit";
      unit.textContent = param.unit;
      group.appendChild(label);
      group.appendChild(input);
      group.appendChild(unit);
      container.appendChild(group);
      input.addEventListener("change", () => {
        if (!currentResult) {
          renderPreview();
        }
      });
    });
  }
  function readPhysicsParams() {
    const params = {};
    parameterConfig.forEach((config) => {
      const input = document.getElementById(config.id);
      if (input && input.value) {
        params[config.physicsKey] = parseFloat(input.value);
      }
    });
    return params;
  }
  function renderPreview() {
    try {
      const { params, sample } = createInitialSample(readPhysicsParams());
      renderer.drawPreview(params, sample);
    } catch (e) {
      console.error("renderPreview failed:", e);
    }
  }
  function setupControls() {
    const pauseBtn = document.getElementById("pauseBtn");
    const resetBtn = document.getElementById("resetBtn");
    const stepBtn = document.getElementById("stepBtn");
    const stepMultiBtn = document.getElementById("stepMultiBtn");
    const playSpeedSlider = document.getElementById("playSpeed");
    const playSpeedValue = document.getElementById("playSpeedValue");
    pauseBtn.addEventListener("click", () => {
      if (!currentResult) {
        fire();
      } else if (paused) {
        paused = false;
        animationStart = performance.now() - pausedTime * 1e3 / playSpeed;
        pauseBtn.textContent = "Pause";
        tick();
      } else {
        paused = true;
        cancelAnimationFrame(animationFrame);
        pauseBtn.textContent = "Play";
      }
    });
    resetBtn.addEventListener("click", () => {
      cancelAnimationFrame(animationFrame);
      currentResult = null;
      paused = true;
      pausedTime = 0;
      pauseBtn.textContent = "Fire";
      clearStats();
      renderPreview();
    });
    stepBtn.addEventListener("click", () => {
      if (!currentResult) fire();
      if (currentResult) {
        pausedTime = Math.min(pausedTime + 1 / 60, currentResult.stats.totalTime);
        paused = true;
        pauseBtn.textContent = "Play";
        renderer.render(currentResult, pausedTime);
        updateStatsAtTime(pausedTime);
      }
    });
    stepMultiBtn.addEventListener("click", () => {
      const stepCount = parseInt(document.getElementById("stepCount").value) || 10;
      if (!currentResult) fire();
      if (currentResult) {
        pausedTime = Math.min(pausedTime + stepCount / 60, currentResult.stats.totalTime);
        paused = true;
        pauseBtn.textContent = "Play";
        renderer.render(currentResult, pausedTime);
        updateStatsAtTime(pausedTime);
      }
    });
    if (playSpeedSlider) {
      playSpeedSlider.addEventListener("input", () => {
        playSpeed = parseFloat(playSpeedSlider.value);
        playSpeedValue.textContent = playSpeed + "x";
        if (!paused && currentResult) {
          animationStart = performance.now() - pausedTime * 1e3 / playSpeed;
        }
      });
    }
    document.addEventListener("keydown", (e) => {
      if (e.code === "Space") {
        e.preventDefault();
        pauseBtn.click();
      } else if (e.code === "Enter") {
        e.preventDefault();
        resetBtn.click();
      } else if (e.code === "ArrowRight") {
        e.preventDefault();
        stepBtn.click();
      }
    });
  }
  function fire() {
    cancelAnimationFrame(animationFrame);
    try {
      const params = readPhysicsParams();
      currentResult = simulateTrebuchet(params);
      renderer.setSimulation(currentResult);
      updateFinalStats(currentResult);
      animationStart = performance.now();
      pausedTime = 0;
      paused = false;
      document.getElementById("pauseBtn").textContent = "Pause";
      tick();
    } catch (error) {
      const msg = error instanceof Error ? error.message : "Unknown error";
      console.error("Simulation error:", msg);
      currentResult = null;
      renderPreview();
    }
  }
  function tick() {
    if (!currentResult || paused) return;
    const elapsed = (performance.now() - animationStart) / 1e3 * playSpeed;
    const displayTime = Math.min(elapsed, currentResult.stats.totalTime);
    pausedTime = displayTime;
    renderer.render(currentResult, displayTime);
    updateStatsAtTime(displayTime);
    if (displayTime < currentResult.stats.totalTime) {
      animationFrame = requestAnimationFrame(tick);
    } else {
      paused = true;
      document.getElementById("pauseBtn").textContent = "Fire";
    }
  }
  function updateStatsAtTime(time) {
    if (!currentResult) return;
    const sample = findSampleAtTime(currentResult.samples, time);
    const params = currentResult.params;
    setStatValue("distanceValue", Math.abs(sample.projectileX - currentResult.samples[0].projectileX).toFixed(1) + " m");
    setStatValue("currentHeightValue", (params.h - sample.projectileY).toFixed(1) + " m");
    setStatValue("velocityValue", sample.projectileSpeed.toFixed(1) + " m/s");
    setStatValue("timeValue", time.toFixed(2) + " s");
    setStatValue("maxDistanceValue", currentResult.stats.range.toFixed(1) + " m");
    setStatValue("heightValue", currentResult.stats.maxHeight.toFixed(1) + " m");
    setStatValue("maxVelocityValue", currentResult.stats.peakSpeed.toFixed(1) + " m/s");
    setStatValue("estimatedDistanceValue", currentResult.stats.range.toFixed(1) + " m");
  }
  function updateFinalStats(result) {
    setStatValue("maxDistanceValue", result.stats.range.toFixed(1) + " m");
    setStatValue("heightValue", result.stats.maxHeight.toFixed(1) + " m");
    setStatValue("maxVelocityValue", result.stats.peakSpeed.toFixed(1) + " m/s");
    setStatValue("estimatedDistanceValue", result.stats.range.toFixed(1) + " m");
  }
  function clearStats() {
    const ids = [
      "distanceValue",
      "currentHeightValue",
      "velocityValue",
      "timeValue",
      "maxDistanceValue",
      "heightValue",
      "maxVelocityValue",
      "estimatedDistanceValue"
    ];
    ids.forEach((id) => setStatValue(id, "0"));
  }
  function setStatValue(id, value) {
    const el = document.getElementById(id);
    if (el) el.textContent = value;
  }
})();
