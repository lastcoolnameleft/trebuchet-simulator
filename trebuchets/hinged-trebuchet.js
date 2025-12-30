// Hinged Counterweight Trebuchet
// Traditional design with a hinged counterweight

class HingedTrebuchetBuilder extends BaseTrebuchetBuilder {
    build(baseX, baseY, params) {
        const bodies = [];
        const joints = [];
        const shortArmRatio = 0.3; // Short arm is 30% of total length
        const longArmRatio = 0.7;  // Long arm is 70% of total length

        // Vertical frame (exact copy from planck.html)
        const frameHeight = 11.25;
        const frameY = baseY - frameHeight;
        const frame = this.simulator.world.createBody({
            position: planck.Vec2(baseX, frameY)
        });
        frame.createFixture({
            shape: planck.Box(0.5, frameHeight),
            friction: 0.5,
            userData: { color: '#8B4513' }
        });
        bodies.push(frame);

        // Throwing arm (pivots 15m above ground)
        const armWidth = 0.75;
        const pivotY = baseY - 15;
        const armCenterOffset = -(params.armLength * longArmRatio - params.armLength / 2);
        const arm = this.simulator.world.createBody({
            position: planck.Vec2(baseX, pivotY),
            type: 'dynamic'
        });
        arm.createFixture({
            shape: planck.Box(params.armLength / 2, armWidth / 2, planck.Vec2(armCenterOffset, 0), 0),
            density: params.armMass / (params.armLength * armWidth),
            friction: 0.5,
            userData: { color: '#D2691E' }
        });
        bodies.push(arm);

        // Pivot joint at fulcrum
        const pivotJoint = this.simulator.world.createJoint(planck.RevoluteJoint({
            bodyA: frame,
            bodyB: arm,
            localAnchorA: planck.Vec2(0, frameHeight / 2 - 0.5),
            localAnchorB: planck.Vec2(0, 0)
        }));
        joints.push(pivotJoint);

        // Counterweight
        const cwX = baseX - (params.armLength * longArmRatio);
        const cwY = pivotY - 1;
        const counterweight = this.createCounterweight(
            cwX, cwY,
            params.counterweightSize,
            params.counterweightMass
        );
        bodies.push(counterweight);

        // Hinge for counterweight
        const cwHinge = this.createCounterweightHinge(
            arm,
            counterweight,
            planck.Vec2(-(params.armLength * longArmRatio), 0),
            1
        );
        joints.push(cwHinge);

        // Projectile
        const projX = baseX + (params.armLength * shortArmRatio);
        const projY = pivotY - params.slingLength;
        const projectile = this.createProjectile(projX, projY, params);
        bodies.push(projectile);

        // Sling
        const slingJoint = this.createSling(
            arm,
            projectile,
            planck.Vec2((params.armLength * shortArmRatio), 0),
            params.slingLength
        );
        joints.push(slingJoint);

        return {
            bodies,
            joints,
            projectile,
            slingJoint
        };
    }
}
