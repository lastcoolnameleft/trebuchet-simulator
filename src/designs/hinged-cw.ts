import { computeTrebuchetGeometry } from '../geometry';
import { simulateTrebuchet, createInitialSample, type TrebuchetParams, type SimulationSample } from '../physics';
import type { TrebuchetDesign, ParameterConfig } from './types';

const parameterConfig: ParameterConfig[] = [
  { id: 'projectileArmLength', physicsKey: 'LAl', label: 'Arm Length (Projectile)', unit: 'm', step: 0.1, min: 0.5, max: 50, default: 2.07 },
  { id: 'counterweightArmLength', physicsKey: 'LAs', label: 'Arm Length (Counterweight)', unit: 'm', step: 0.1, min: 0.1, max: 20, default: 0.533 },
  { id: 'armHeight', physicsKey: 'h', label: 'Pivot Height', unit: 'm', step: 0.1, min: 0.5, max: 40, default: 1.524 },
  { id: 'counterweightMass', physicsKey: 'mW', label: 'Counterweight Mass', unit: 'kg', step: 1, min: 1, max: 5000, default: 44.49 },
  { id: 'cwHangLength', physicsKey: 'LW', label: 'CW Hanging Length', unit: 'm', step: 0.1, min: 0.1, max: 10, default: 0.61 },
  { id: 'projectileMass', physicsKey: 'mP', label: 'Projectile Mass', unit: 'kg', step: 0.01, min: 0.01, max: 200, default: 0.149 },
  { id: 'slingLength', physicsKey: 'LS', label: 'Sling Length', unit: 'm', step: 0.1, min: 0.1, max: 30, default: 2.08 },
  { id: 'armMass', physicsKey: 'mA', label: 'Arm Mass', unit: 'kg', step: 0.5, min: 0.5, max: 500, default: 4.83 },
  { id: 'releaseAngle', physicsKey: 'releaseAngle', label: 'Release Angle', unit: '°', step: 1, min: 10, max: 80, default: 45 },
];

export const hingedCW: TrebuchetDesign = {
  id: 'hinged',
  name: 'Hinged Counterweight',
  parameterConfig,
  simulate: simulateTrebuchet,
  createInitialSample,
  computeGeometry(params: TrebuchetParams, sample: SimulationSample) {
    return computeTrebuchetGeometry(params, sample);
  },
};
