import { floatingArm } from '../src/designs/floating-arm';

console.log('=== FAT 5DOF Verification ===\n');

// Use reference model default parameters (Constans/Rowan University)
const result = floatingArm.simulate({
  LAl: 0.4, LAs: 0.15, LS: 0.2, h: 0.4, mW: 10, mP: 0.1,
  releaseAngle: -65, pinHeight: 0.2, pinDistance: 0.2, rampAngle: 10,
} as any);

console.log('FAT (Floating Arm) with reference params:');
console.log(`  Range: ${result.stats.range.toFixed(2)} m`);
console.log(`  Release speed: ${result.stats.releaseSpeed.toFixed(2)} m/s`);
console.log(`  Release angle: ${result.stats.releaseAngle?.toFixed(1)}°`);
console.log(`  Release time: ${result.stats.releaseTime.toFixed(4)} s`);
console.log(`  Flight time: ${result.stats.flightTime.toFixed(4)} s`);
console.log(`  Total time: ${result.stats.totalTime.toFixed(4)} s`);
console.log(`  Samples: ${result.samples.length}`);

console.log('\n--- Sanity Checks ---');
let allPass = true;

function check(name: string, cond: boolean, msg: string) {
  console.log(`  ${cond ? '✅' : '❌'} ${name}: ${msg}`);
  if (!cond) allPass = false;
}

check('Range > 5m', result.stats.range > 5, `${result.stats.range.toFixed(1)} m`);
check('Range < 100m', result.stats.range < 100, `${result.stats.range.toFixed(1)} m`);
check('Release speed > 5 m/s', result.stats.releaseSpeed > 5, `${result.stats.releaseSpeed.toFixed(1)} m/s`);
check('Release time < 1s', result.stats.releaseTime < 1, `${result.stats.releaseTime.toFixed(3)} s`);
check('Total time > 0.5s', result.stats.totalTime > 0.5, `${result.stats.totalTime.toFixed(3)} s`);
check('Flight time > 0.5s', result.stats.flightTime > 0.5, `${result.stats.flightTime.toFixed(3)} s`);
check('Max height > 1m', result.stats.maxHeight > 1, `${result.stats.maxHeight.toFixed(2)} m`);

// Geometry check
const init = floatingArm.createInitialSample({
  LAl: 0.4, LAs: 0.15, LS: 0.2, h: 0.4, mW: 10, mP: 0.1,
  releaseAngle: -65, pinHeight: 0.2, pinDistance: 0.2, rampAngle: 10,
} as any);
const geom = floatingArm.computeGeometry(init.params, init.sample);
check('CW at top (y=0 in renderer)', Math.abs(geom.counterweight.y) < 0.01, `CW y=${geom.counterweight.y.toFixed(4)}`);
check('Arm tip below CW', geom.slingAttach.y > geom.counterweight.y, `tip y=${geom.slingAttach.y.toFixed(4)}`);
check('Projectile below arm tip', geom.projectile.y >= geom.slingAttach.y - 0.01, `proj y=${geom.projectile.y.toFixed(4)}`);

console.log(`\n${allPass ? '✅ ALL CHECKS PASSED' : '❌ SOME CHECKS FAILED'}`);
process.exit(allPass ? 0 : 1);
