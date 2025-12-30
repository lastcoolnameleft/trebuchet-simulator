// Sandbox Trebuchet
// Based on planck.html working implementation
// Customizable starting point for experimentation

class SandboxTrebuchetBuilder extends BaseTrebuchetBuilder {
    build(baseX, baseY, params) {
        const bodies = [];
        const joints = [];
        const leftArmLength = params.armLength * 0.7;
        const rightArmLength = params.armLength * 0.3;
        const armCenterOffset = (rightArmLength - leftArmLength) / 2;

        // Vertical frame - height adjusts with arm height
        const armHeight = params.armHeight || 22.5; // Customizable arm height (default: top of frame)
        const frameHeight = armHeight / 2; // Half-height for Box shape
        const frameY = baseY - frameHeight; // Center frame between ground and pivot
        const frame = this.simulator.world.createBody({
            position: planck.Vec2(baseX, frameY)
        });
        frame.createFixture({
            shape: planck.Box(0.5, frameHeight),
            friction: 0.5,
            filterCategoryBits: 0x0002,
            filterMaskBits: 0x0001,
            userData: { color: '#8B4513' }
        });
        bodies.push(frame);

        // Catapult arm - 70/30 split with offset center
        const pivotY = baseY - armHeight;
        const arm = this.simulator.world.createBody({
            position: planck.Vec2(baseX, pivotY),
            type: 'dynamic',
            angle: -Math.PI / 4 // 45 degrees (from planck.html)
        });
        arm.createFixture({
            shape: planck.Box(params.armLength / 2, 1 / 2, planck.Vec2(armCenterOffset, 0), 0),
            density: 1.0,
            friction: 0.5,
            userData: { color: '#D2691E' }
        });
        bodies.push(arm);

        // Pivot joint (from planck.html)
        const pivotJoint = this.simulator.world.createJoint(planck.RevoluteJoint({
            bodyA: frame,
            bodyB: arm,
            localAnchorA: planck.Vec2(0, pivotY - frameY), // Relative to frame center
            localAnchorB: planck.Vec2(0, 0),
            enableLimit: false
        }));
        joints.push(pivotJoint);

        // Counterweight - calculate position based on arm angle and attachment length
        const cwRadius = 2.5;
        const cwAttachLength = 5;
        const armAngle = -Math.PI / 4; // Same angle as arm
        
        // Calculate the world position of the counterweight attachment point on the arm
        const cwAttachX = baseX + (rightArmLength * Math.cos(armAngle));
        const cwAttachY = pivotY + (rightArmLength * Math.sin(armAngle));
        
        // Position counterweight exactly cwAttachLength away, hanging below attachment point
        const cwX = cwAttachX;
        const cwY = cwAttachY + cwAttachLength;
        
        // Verify distance
        const cwActualDistance = Math.sqrt(
            Math.pow(cwX - cwAttachX, 2) + Math.pow(cwY - cwAttachY, 2)
        );
        console.log(`Counterweight attach length: ${cwAttachLength}, Actual distance: ${cwActualDistance.toFixed(3)}`);
        
        const counterweight = this.simulator.world.createBody({
            position: planck.Vec2(cwX, cwY),
            type: 'dynamic',
            linearDamping: 0.1,
            angularDamping: 0.1
        });
        counterweight.createFixture({
            shape: planck.Circle(cwRadius),
            density: 5.0,
            friction: 0.5,
            filterCategoryBits: 0x0004,
            filterMaskBits: 0x0001,
            userData: { color: '#696969' }
        });
        // Ensure counterweight starts at rest
        counterweight.setLinearVelocity(planck.Vec2(0, 0));
        counterweight.setAngularVelocity(0);
        bodies.push(counterweight);

        // Counterweight hinge (distance joint from planck.html)
        const cwHinge = this.simulator.world.createJoint(planck.DistanceJoint({
            bodyA: arm,
            bodyB: counterweight,
            localAnchorA: planck.Vec2(rightArmLength, 0),
            localAnchorB: planck.Vec2(0, 0),
            length: cwAttachLength,
            dampingRatio: 0.4,
            frequencyHz: 8.0
        }));
        joints.push(cwHinge);

        // Projectile - calculate position based on arm angle and sling length
        const projRadius = 0.75;
        const slingLength = params.slingLength || 6;
        
        // Calculate the world position of the sling attachment point on the arm (accounting for rotation)
        // Reuse armAngle from above
        // The attachment is at -leftArmLength along the arm (in local coordinates)
        // Convert to world coordinates considering the arm's rotation
        const slingAttachX = baseX + (-leftArmLength * Math.cos(armAngle));
        const slingAttachY = pivotY + (-leftArmLength * Math.sin(armAngle));
        
        // Position projectile on ground, with slack extending to the right if needed
        const projY = baseY - projRadius; // On ground surface
        const verticalDistance = Math.abs(projY - slingAttachY);
        
        // Calculate horizontal position to maintain exact sling length
        let projX;
        if (slingLength > verticalDistance) {
            const horizontalSlack = Math.sqrt(slingLength * slingLength - verticalDistance * verticalDistance);
            projX = slingAttachX + horizontalSlack; // Extend to the right
        } else {
            projX = slingAttachX; // Directly below if sling is shorter
        }
        
        // Verify distance (for debugging)
        const actualDistance = Math.sqrt(
            Math.pow(projX - slingAttachX, 2) + Math.pow(projY - slingAttachY, 2)
        );
        console.log(`Sling length: ${slingLength}, Actual distance: ${actualDistance.toFixed(3)}`);
        
        const projectile = this.simulator.world.createBody({
            position: planck.Vec2(projX, projY),
            type: 'dynamic',
            linearDamping: 0.1,
            angularDamping: 0.1
        });
        projectile.createFixture({
            shape: planck.Circle(projRadius),
            density: 1.0,
            friction: 0.5,
            restitution: 0.0, // No bounce
            filterCategoryBits: 0x0008,
            filterMaskBits: 0x0001 | 0x0004, // Collide with ground and counterweight, but not frame (0x0002)
            userData: { color: '#FF4500' }
        });
        // Ensure projectile starts at rest
        projectile.setLinearVelocity(planck.Vec2(0, 0));
        projectile.setAngularVelocity(0);
        bodies.push(projectile);

        // Sling (distance joint - soft enough to prevent wild initial forces)
        const slingJoint = this.simulator.world.createJoint(planck.DistanceJoint({
            bodyA: arm,
            bodyB: projectile,
            localAnchorA: planck.Vec2(-leftArmLength, 0),
            localAnchorB: planck.Vec2(0, 0),
            length: slingLength,
            dampingRatio: 0.5,  // Lower damping to allow more give
            frequencyHz: 3.0    // Much softer spring
        }));
        joints.push(slingJoint);

        return {
            bodies,
            joints,
            projectile,
            slingJoint
        };
    }
}
