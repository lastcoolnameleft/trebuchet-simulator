export interface Point {
  x: number;
  y: number;
}

export interface TrebuchetPose {
  Aq: number;
  Wq: number;
  Sq: number;
}

export interface GeometryParams {
  LAl: number;
  LAs: number;
  LAcg: number;
  LW: number;
  LS: number;
}

export interface TrebuchetGeometry {
  armCg: Point;
  counterweightAttach: Point;
  counterweight: Point;
  slingAttach: Point;
  projectile: Point;
}

export function computeTrebuchetGeometry(params: GeometryParams, pose: TrebuchetPose): TrebuchetGeometry {
  const { Aq, Wq, Sq } = pose;
  const { LAl, LAs, LAcg, LW, LS } = params;

  // All positions relative to pivot, y-positive-DOWN (matches renderer's worldToScreen)
  // VT convention: Y_up = ... → renderer: y_down = -Y_up
  return {
    armCg: {
      x: -LAcg * Math.sin(Aq),
      y: -LAcg * Math.cos(Aq),
    },
    counterweightAttach: {
      x: LAs * Math.sin(Aq),
      y: LAs * Math.cos(Aq),
    },
    counterweight: {
      x: LAs * Math.sin(Aq) + LW * Math.sin(Aq + Wq),
      y: LAs * Math.cos(Aq) + LW * Math.cos(Aq + Wq),
    },
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

export function computeProjectileVelocity(
  params: Pick<GeometryParams, 'LAl' | 'LS'>,
  pose: Pick<TrebuchetPose, 'Aq' | 'Sq'>,
  angularVelocity: { Aw: number; Sw: number },
): Point {
  const theta = pose.Aq + pose.Sq;
  const thetaDot = angularVelocity.Aw + angularVelocity.Sw;

  return {
    x: -params.LAl * Math.cos(pose.Aq) * angularVelocity.Aw - params.LS * Math.cos(theta) * thetaDot,
    y: -params.LAl * Math.sin(pose.Aq) * angularVelocity.Aw - params.LS * Math.sin(theta) * thetaDot,
  };
}

export function computeProjectileAcceleration(
  params: Pick<GeometryParams, 'LAl' | 'LS'>,
  pose: Pick<TrebuchetPose, 'Aq' | 'Sq'>,
  angularVelocity: { Aw: number; Sw: number },
  angularAcceleration: { Aacc: number; Sacc: number },
): Point {
  const theta = pose.Aq + pose.Sq;
  const thetaDot = angularVelocity.Aw + angularVelocity.Sw;
  const thetaAcc = angularAcceleration.Aacc + angularAcceleration.Sacc;

  return {
    x:
      params.LAl * Math.sin(pose.Aq) * angularVelocity.Aw ** 2 -
      params.LAl * Math.cos(pose.Aq) * angularAcceleration.Aacc +
      params.LS * Math.sin(theta) * thetaDot ** 2 -
      params.LS * Math.cos(theta) * thetaAcc,
    y:
      -params.LAl * Math.cos(pose.Aq) * angularVelocity.Aw ** 2 -
      params.LAl * Math.sin(pose.Aq) * angularAcceleration.Aacc -
      params.LS * Math.cos(theta) * thetaDot ** 2 -
      params.LS * Math.sin(theta) * thetaAcc,
  };
}

export function computeCounterweightVelocity(
  params: Pick<GeometryParams, 'LAs' | 'LW'>,
  pose: Pick<TrebuchetPose, 'Aq' | 'Wq'>,
  angularVelocity: { Aw: number; Ww: number },
): Point {
  const theta = pose.Aq + pose.Wq;
  const thetaDot = angularVelocity.Aw + angularVelocity.Ww;

  return {
    x: params.LAs * Math.cos(pose.Aq) * angularVelocity.Aw + params.LW * Math.cos(theta) * thetaDot,
    y: params.LAs * Math.sin(pose.Aq) * angularVelocity.Aw + params.LW * Math.sin(theta) * thetaDot,
  };
}

export function computeArmCgVelocity(
  params: Pick<GeometryParams, 'LAcg'>,
  pose: Pick<TrebuchetPose, 'Aq'>,
  angularVelocity: { Aw: number },
): Point {
  return {
    x: params.LAcg * Math.cos(pose.Aq) * angularVelocity.Aw,
    y: params.LAcg * Math.sin(pose.Aq) * angularVelocity.Aw,
  };
}
