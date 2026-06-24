import type { TrebuchetGeometry } from '../geometry';
import type { SimulationResult, SimulationSample, TrebuchetParams } from '../physics';

export interface ParameterConfig {
  id: string;
  physicsKey: string;
  label: string;
  unit: string;
  step: number;
  min: number;
  max: number;
  default: number;
}

export type GeometryFunction = (params: TrebuchetParams, sample: SimulationSample) => TrebuchetGeometry;

export interface TrebuchetDesign {
  id: string;
  name: string;
  parameterConfig: ParameterConfig[];
  simulate(input: Partial<TrebuchetParams>): SimulationResult;
  createInitialSample(input: Partial<TrebuchetParams>): { params: TrebuchetParams; sample: SimulationSample };
  computeGeometry: GeometryFunction;
}
