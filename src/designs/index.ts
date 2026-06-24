import type { TrebuchetDesign } from './types';
import { hingedCW } from './hinged-cw';
import { fixedCW } from './fixed-cw';
import { floatingArm } from './floating-arm';

export type { TrebuchetDesign, ParameterConfig, GeometryFunction } from './types';

export const designs: Record<string, TrebuchetDesign> = {
  [hingedCW.id]: hingedCW,
  [fixedCW.id]: fixedCW,
  [floatingArm.id]: floatingArm,
};

export const defaultDesignId = hingedCW.id;
