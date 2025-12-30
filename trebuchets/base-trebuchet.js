// Base Trebuchet Builder
// Abstract base class for all trebuchet types

class BaseTrebuchetBuilder {
    constructor(simulator) {
        this.simulator = simulator;
        this.SCALE = 20; // pixels per meter
    }

    // Common method to create projectile
    createProjectile(x, y, params) {
        const projectileRadius = params.projectileSize;
        const projectile = this.simulator.world.createBody({
            position: planck.Vec2(x, y),
            type: 'dynamic'
        });
        projectile.createFixture({
            shape: planck.Circle(projectileRadius),
            density: params.projectileMass / (Math.PI * projectileRadius * projectileRadius),
            restitution: 0.6,
            friction: 0.5,
            userData: { color: '#FF4500' }
        });
        return projectile;
    }

    // Common method to create sling joint
    createSling(arm, projectile, slingPointLocal, slingLength) {
        const slingJoint = this.simulator.world.createJoint(planck.DistanceJoint({
            bodyA: arm,
            bodyB: projectile,
            localAnchorA: slingPointLocal,
            localAnchorB: planck.Vec2(0, 0),
            length: slingLength,
            dampingRatio: 0.1,
            frequencyHz: 2.0
        }));
        return slingJoint;
    }

    // Common method to create counterweight
    createCounterweight(x, y, size, mass) {
        const cwSize = size;
        const counterweight = this.simulator.world.createBody({
            position: planck.Vec2(x, y),
            type: 'dynamic'
        });
        counterweight.createFixture({
            shape: planck.Box(cwSize / 2, cwSize / 2),
            density: mass / (cwSize * cwSize),
            friction: 0.5,
            filterCategoryBits: 0x0004,
            filterMaskBits: 0x0001 | 0x0004 | 0x0008,
            userData: { color: '#696969' }
        });
        return counterweight;
    }

    // Common method to create counterweight hinge
    createCounterweightHinge(arm, counterweight, attachPointLocal, cwHingeLength, dampingRatio = 0.4) {
        const cwHinge = this.simulator.world.createJoint(planck.DistanceJoint({
            bodyA: arm,
            bodyB: counterweight,
            localAnchorA: attachPointLocal,
            localAnchorB: planck.Vec2(0, 0),
            length: cwHingeLength,
            dampingRatio: dampingRatio,
            frequencyHz: 8.0
        }));
        return cwHinge;
    }

    // Abstract method - must be implemented by subclasses
    build(baseX, baseY, params) {
        throw new Error('build() must be implemented by subclass');
    }
}
