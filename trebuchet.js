// Trebuchet Simulator using Planck.js
// Main simulator class that coordinates physics and trebuchet construction

const SCALE = 20; // pixels per meter

class TrebuchetSimulator {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.trebuchetType = 'sandbox';
        this.parameters = this.getDefaultParameters();
        this.fired = false;
        this.paused = true; // Start paused
        this.stats = { distance: 0, height: 0, maxDistance: 0, maxHeight: 0, velocity: 0, maxVelocity: 0, time: 0, estimatedDistance: 0 };
        this.projectileTrajectory = [];
        this.animationId = null;
        this.projectileHitGround = false; // Track if projectile hit ground
        this.cameraScale = SCALE; // Dynamic zoom level
        this.cameraOffsetX = 0; // Dynamic camera offset
        this.trebuchetX = 0; // Track trebuchet position for camera
        this.setupPhysics();
        // Initialize trebuchet builders
        this.builders = {
            fixed: new FixedCounterweightTrebuchetBuilder(this),
            hinged: new HingedCounterweightTrebuchetBuilder(this),
            whipper: new WhipperTrebuchetBuilder(this),
            floating: new FloatingArmTrebuchetBuilder(this),
            walking: new WalkingArmTrebuchetBuilder(this),
            sandbox: new SandboxTrebuchetBuilder(this)
        };
    }

    setupPhysics() {
        // Create world with Planck.js
        this.world = planck.World({
            gravity: planck.Vec2(0, 10) // 10 m/s² gravity
        });
        
        // Setup collision listener to detect projectile hitting ground
        this.world.on('begin-contact', (contact) => {
            const fixtureA = contact.getFixtureA();
            const fixtureB = contact.getFixtureB();
            const bodyA = fixtureA.getBody();
            const bodyB = fixtureB.getBody();
            
            // Debug: log ALL collisions to see what's happening
            const userDataA = bodyA.getUserData();
            const userDataB = bodyB.getUserData();
            console.log('🔵 COLLISION in trebuchet.js:', 
                userDataA?.name || 'unknown', 'vs', userDataB?.name || 'unknown',
                '\n  bodyA === this.projectile:', bodyA === this.projectile,
                '\n  bodyB === this.projectile:', bodyB === this.projectile,
                '\n  bodyA === this.ground:', bodyA === this.ground,
                '\n  bodyB === this.ground:', bodyB === this.ground,
                '\n  this.fired:', this.fired,
                '\n  fixtureA category:', fixtureA.getFilterCategoryBits().toString(16),
                'mask:', fixtureA.getFilterMaskBits().toString(16),
                '\n  fixtureB category:', fixtureB.getFilterCategoryBits().toString(16),
                'mask:', fixtureB.getFilterMaskBits().toString(16),
                '\n  bodyA type:', bodyA.getType(),
                'bullet:', bodyA.isBullet(),
                '\n  bodyB type:', bodyB.getType(),
                'bullet:', bodyB.isBullet(),
                '\n  Projectile pos:', this.projectile?.getPosition());
            
            // Check if projectile hit ground (mark for stats tracking)
            // Only count if projectile has been launched
            if ((bodyA === this.projectile && bodyB === this.ground) ||
                (bodyB === this.projectile && bodyA === this.ground)) {
                if (!this.projectileHitGround && this.fired) {
                    this.projectileHitGround = true;
                    console.log('🎯 PROJECTILE HIT GROUND - STOPPING');
                    // Stop the projectile immediately
                    if (this.projectile) {
                        this.projectile.setLinearVelocity(planck.Vec2(0, 0));
                        this.projectile.setAngularVelocity(0);
                        this.projectile.setType('static');
                    }
                }
            }
        });
        
        // Canvas dimensions
        const rect = this.canvas.getBoundingClientRect();
        this.canvas.width = rect.width;
        this.canvas.height = rect.height;

        // Estimate throw distance to size the world appropriately
        const estimatedRange = this.estimateThrowDistance();
        const worldWidth = Math.max(200, estimatedRange * 3); // 3x estimated range for safety
        
        console.log(`🌍 World size: ${worldWidth.toFixed(0)}m (based on estimated range: ${estimatedRange.toFixed(0)}m)`);

        // Create ground (thin ground layer)
        // Use fixed world coordinates - ground at Y = 100m
        const groundHalfHeight = 0.5; // Thinner ground
        const groundHalfWidth = worldWidth * 2; // Make ground 4x world width to catch far projectiles
        const groundCenterX = worldWidth / 2; // Center the ground in the world
        const groundCenterY = 100.0; // Fixed world Y position
        const groundBody = this.world.createBody({
            position: planck.Vec2(groundCenterX, groundCenterY),
            type: 'static',
            userData: { name: 'ground' } // Debug label
        });
        groundBody.createFixture({
            shape: planck.Box(groundHalfWidth, groundHalfHeight),
            friction: 100.0, // Very high friction to stop projectile quickly
            restitution: 0.0, // No bounce
            filterCategoryBits: 0x0001, // Ground category
            filterMaskBits: 0xFFFF, // Collide with everything
            userData: { color: '#654321', name: 'ground' }
        });
        this.ground = groundBody;
        // Calculate ground top surface: ground center Y minus half-height
        this.groundTop = groundCenterY - groundHalfHeight;
        
        // Debug ground properties
        console.log('🏗️ GROUND CREATED:',
            '\n  Position:', groundBody.getPosition(),
            '\n  Type:', groundBody.getType(),
            '\n  Fixture category:', groundBody.getFixtureList().getFilterCategoryBits().toString(16),
            '\n  Fixture mask:', groundBody.getFixtureList().getFilterMaskBits().toString(16),
            '\n  Ground top:', this.groundTop);

        // Start animation loop
        this.animate();
    }

    checkSlingRelease() {
        // Check for sling release based on projectile angle to ground
        // Release when projectile is at 45° to the horizontal for optimal trajectory
        if (this.slingJoint && this.projectile) {
            const projPos = this.projectile.getPosition();
            
            // Calculate angle from horizontal
            // Projectile velocity direction is a good proxy for release angle
            const vel = this.projectile.getLinearVelocity();
            const velocityAngle = Math.atan2(-vel.y, vel.x); // -y because y increases downward
            
            // Release at 45° (π/4 radians) for optimal trajectory
            const targetReleaseAngle = Math.PI / 4; // 45 degrees
            const angleTolerance = 0.05; // Small tolerance window
            
            // Only release if projectile has significant velocity and is near 45°
            const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y);
            if (speed > 10 && Math.abs(velocityAngle - targetReleaseAngle) < angleTolerance) {
                this.world.destroyJoint(this.slingJoint);
                this.slingJoint = null;
                this.fired = true;
                console.log('🎯 Sling released! Projectile angle:', (velocityAngle * 180 / Math.PI).toFixed(1), '° velocity:', speed.toFixed(1), 'm/s');
            }
        }
    }

    updateProjectileTracking() {
        // Track projectile
        if (this.projectile) {
            // Keep projectile stopped if it already hit ground
            if (this.projectileHitGround) {
                this.projectile.setLinearVelocity(planck.Vec2(0, 0));
                this.projectile.setAngularVelocity(0);
                if (this.projectile.getType() !== 'static') {
                    this.projectile.setType('static');
                }
            }
            
            const pos = this.projectile.getPosition();
            this.projectileTrajectory.push({ x: pos.x * SCALE, y: pos.y * SCALE });
            
            // Update stats - real-time accurate
            // Distance from starting X position (in meters)
            const distance = pos.x - this.startX;
            
            // Height above starting position (Y increases downward, so subtract)
            const height = this.startY - pos.y;
            
            // Update max distance (always show current distance if projectile moved right)
            this.stats.distance = Math.max(0, distance);
            
            // Track current and max distance
            if (distance > this.stats.maxDistance) {
                this.stats.maxDistance = distance;
            }
            
            // Track current and max height
            this.stats.height = Math.max(0, height);
            if (height > this.stats.maxHeight) {
                this.stats.maxHeight = height;
            }
            
            // Track velocity (current and max)
            const vel = this.projectile.getLinearVelocity();
            if (vel) {
                const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y);
                if (!isNaN(speed)) {
                    this.stats.velocity = speed;
                    if (speed > this.stats.maxVelocity) {
                        this.stats.maxVelocity = speed;
                    }
                }
            }
            
            // Track time (only if projectile hasn't hit ground yet)
            if (!this.projectileHitGround) {
                this.stats.time += 1/60;
            }
        }
    }

    animate() {
        if (!this.paused) {
            this.world.step(1/60);
            this.checkSlingRelease();
            this.updateProjectileTracking();
        }
        
        this.render();
        this.animationId = requestAnimationFrame(() => this.animate());
    }

    render() {
        const rect = this.canvas.getBoundingClientRect();
        this.ctx.fillStyle = '#0f0f13';
        this.ctx.fillRect(0, 0, rect.width, rect.height);
        
        // Dynamic camera based on simulation state
        let cameraScale, cameraOffsetX;
        
        if (this.fired) {
            // Animation running: zoom out to show full trajectory
            const estimatedDistance = parseFloat(this.stats.estimatedDistance) || 100;
            const totalWidth = estimatedDistance + 50; // Add some padding
            cameraScale = rect.width / totalWidth; // Scale to fit in canvas width
            cameraOffsetX = 50; // Trebuchet near left edge
        } else {
            // Before animation: zoomed in on trebuchet
            cameraScale = 16; // Closer zoom
            cameraOffsetX = rect.width / 2 - 130; // Center the trebuchet
        }
        
        this.cameraScale = cameraScale;
        this.cameraOffsetX = cameraOffsetX;
        
        // Calculate Y offset to keep ground at bottom of screen
        // Ground's world Y position should always render at rect.height on screen
        const groundWorldY = this.ground.getPosition().y;
        const cameraOffsetY = rect.height - (groundWorldY * cameraScale);
        
        // Draw all bodies
        for (let body = this.world.getBodyList(); body; body = body.getNext()) {
            this.ctx.save();
            const pos = body.getPosition();
            this.ctx.translate(pos.x * cameraScale + cameraOffsetX, pos.y * cameraScale + cameraOffsetY);
            this.ctx.rotate(body.getAngle());
            
            for (let fixture = body.getFixtureList(); fixture; fixture = fixture.getNext()) {
                const shape = fixture.getShape();
                
                if (shape.getType() === 'circle') {
                    this.ctx.fillStyle = fixture.getUserData()?.color || '#8B4513';
                    this.ctx.beginPath();
                    this.ctx.arc(0, 0, shape.getRadius() * cameraScale, 0, Math.PI * 2);
                    this.ctx.fill();
                } else if (shape.getType() === 'polygon') {
                    const vertices = shape.m_vertices;
                    this.ctx.fillStyle = fixture.getUserData()?.color || '#654321';
                    this.ctx.beginPath();
                    this.ctx.moveTo(vertices[0].x * cameraScale, vertices[0].y * cameraScale);
                    for (let i = 1; i < vertices.length; i++) {
                        this.ctx.lineTo(vertices[i].x * cameraScale, vertices[i].y * cameraScale);
                    }
                    this.ctx.closePath();
                    this.ctx.fill();
                }
            }
            this.ctx.restore();
        }
        
        // Draw joints
        this.ctx.strokeStyle = '#8B4513';
        this.ctx.lineWidth = 2;
        for (let joint = this.world.getJointList(); joint; joint = joint.getNext()) {
            const anchorA = joint.getAnchorA();
            const anchorB = joint.getAnchorB();
            
            this.ctx.beginPath();
            this.ctx.moveTo(anchorA.x * cameraScale + cameraOffsetX, anchorA.y * cameraScale + cameraOffsetY);
            this.ctx.lineTo(anchorB.x * cameraScale + cameraOffsetX, anchorB.y * cameraScale + cameraOffsetY);
            this.ctx.stroke();
        }
    }

    estimateThrowDistance() {
        // Estimate throw distance based on trebuchet parameters
        // Using simplified energy conservation and projectile motion
        const params = this.parameters;
        const g = 10; // gravity m/s²
        
        // Energy available from counterweight drop
        // Assume counterweight drops about 70% of counterweight arm length
        const cwDropHeight = params.counterweightArmLength * 0.7; // ~70% of counterweight arm
        const potentialEnergy = params.counterweightMass * g * cwDropHeight;
        
        // Energy transferred to projectile (assume 30% efficiency - typical for trebuchets)
        const efficiency = 0.3;
        const projectileEnergy = potentialEnergy * efficiency;
        
        // Launch velocity: KE = 0.5 * m * v²
        const launchVelocity = Math.sqrt((2 * projectileEnergy) / params.projectileMass);
        
        // Assume optimal release angle of 45° for maximum range
        // Range = v² * sin(2θ) / g, where θ = 45°, sin(90°) = 1
        const releaseHeight = params.armHeight || 13;
        
        // Basic projectile motion with initial height
        // Range ≈ (v² / g) + extra distance from height
        const baseRange = (launchVelocity * launchVelocity) / g;
        const heightBonus = releaseHeight * 2; // Approximate additional distance from height
        const estimatedRange = baseRange + heightBonus;
        
        console.log(`📐 Throw Distance Estimate:
  Counterweight drop: ${cwDropHeight.toFixed(1)}m
  Potential energy: ${potentialEnergy.toFixed(0)}J
  Launch velocity: ${launchVelocity.toFixed(1)}m/s
  Estimated range: ${estimatedRange.toFixed(1)}m`);
        
        return estimatedRange;
    }

    getDefaultParameters() {
        return {
            projectileArmLength: 14,  // Projectile side (formerly 70% of 20)
            counterweightArmLength: 6,  // Counterweight side (formerly 30% of 20)
            counterweightMass: 200,
            counterweightSize: 1,
            projectileMass: 10,
            projectileSize: 0.4,  // diameter (was 0.2 radius)
            slingLength: 12,
            armMass: 30,
            releaseAngle: 45,
            armHeight: 13
        };
    }
    
    getBuilderClass(type) {
        const classMap = {
            fixed: FixedCounterweightTrebuchetBuilder,
            hinged: HingedCounterweightTrebuchetBuilder,
            whipper: WhipperTrebuchetBuilder,
            floating: FloatingArmTrebuchetBuilder,
            walking: WalkingArmTrebuchetBuilder,
            sandbox: SandboxTrebuchetBuilder
        };
        return classMap[type];
    }
    
    buildTrebuchet(type, params) {
        // Clear existing trebuchet
        if (this.trebuchetBodies) {
            this.trebuchetBodies.forEach(body => {
                this.world.destroyBody(body);
            });
        }
        if (this.joints) {
            this.joints.forEach(joint => {
                this.world.destroyJoint(joint);
            });
        }

        this.trebuchetType = type;
        this.parameters = { ...this.getDefaultParameters(), ...params };
        this.fired = false;        this.projectileHitGround = false; // Reset hit ground flag        this.projectileTrajectory = [];
        this.stats = { distance: 0, height: 0, maxDistance: 0, maxHeight: 0, velocity: 0, maxVelocity: 0, time: 0, estimatedDistance: 0 };
        
        // Run silent simulation to calculate estimated distance
        this.calculateEstimatedDistance();

        const rect = this.canvas.getBoundingClientRect();
        const baseX = 200 / SCALE;
        const baseY = this.groundTop; // Use stored ground top position
        
        // Store trebuchet position for camera tracking
        this.trebuchetX = baseX;

        // Use the appropriate builder
        const builder = this.builders[type];
        if (!builder) {
            console.error(`Unknown trebuchet type: ${type}`);
            return;
        }

        const result = builder.build(baseX, baseY, this.parameters);
        
        // Store results
        this.trebuchetBodies = result.bodies;
        this.joints = result.joints;
        this.projectile = result.projectile;
        this.slingJoint = result.slingJoint;
        const projPos = this.projectile.getPosition();
        this.startX = projPos.x;
        this.startY = projPos.y;
    }

    fire() {
        if (this.fired || !this.projectile) return;
        
        // Unpause if paused
        if (this.paused) {
            this.paused = false;
        }
        
        this.fired = true;
        this.stats = { distance: 0, height: 0, maxDistance: 0, maxHeight: 0, velocity: 0, maxVelocity: 0, time: 0 };
        
        // Release sling immediately or at appropriate angle
        setTimeout(() => {
            if (this.slingJoint) {
                this.world.destroyJoint(this.slingJoint);
                this.slingJoint = null;
            }
        }, 100);
    }

    reset() {
        this.buildTrebuchet(this.trebuchetType, this.parameters);
        // Pause simulation after reset so it doesn't auto-start
        this.paused = true;
    }

    pause() {
        this.paused = !this.paused;
        return this.paused;
    }

    step(numSteps = 1) {
        // Ensure simulation is paused
        const wasPaused = this.paused;
        this.paused = true;
        
        // Step forward the specified number of frames
        for (let i = 0; i < numSteps; i++) {
            this.world.step(1/60);
            this.checkSlingRelease();
            this.updateProjectileTracking();
        }
        
        // Render after stepping
        this.render();
        
        // Update stats display
        if (window.updateStats) {
            window.updateStats(this.getStats());
        }
        
        //console.log(`⏭️  Stepped forward ${numSteps} frame(s)`);
    }

    updateParameter(param, value) {
        this.parameters[param] = parseFloat(value);
        this.reset();
    }

    calculateEstimatedDistance() {
        // Create a temporary world for silent simulation
        const tempWorld = planck.World({
            gravity: planck.Vec2(0, 10)
        });
        
        // Create temporary simulator context
        const tempSimulator = {
            world: tempWorld,
            ground: this.ground
        };
        
        // Get builder and rebuild trebuchet in temp world
        const builderClass = this.getBuilderClass(this.trebuchetType);
        const builder = new builderClass(tempSimulator);
        const baseX = 10;
        const result = builder.build(baseX, this.groundTop, this.parameters);
        
        const tempProjectile = result.projectile;
        const tempSlingJoint = result.slingJoint;
        let tempFired = false;
        let tempProjectileHitGround = false;
        let maxX = 0;
        let simTime = 0;
        const maxSimTime = 30; // Run for max 30 seconds
        const dt = 1/60; // 60 FPS
        
        // Run simulation silently
        while (simTime < maxSimTime && !tempProjectileHitGround) {
            tempWorld.step(dt);
            simTime += dt;
            
            // Check for sling release
            if (tempSlingJoint && tempProjectile && !tempFired) {
                const vel = tempProjectile.getLinearVelocity();
                const velocityAngle = Math.atan2(-vel.y, vel.x);
                const targetReleaseAngle = Math.PI / 4;
                const angleTolerance = 0.05;
                const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y);
                
                if (speed > 10 && Math.abs(velocityAngle - targetReleaseAngle) < angleTolerance) {
                    tempWorld.destroyJoint(tempSlingJoint);
                    tempFired = true;
                }
            }
            
            // Track max distance
            if (tempProjectile && tempFired) {
                const pos = tempProjectile.getPosition();
                maxX = Math.max(maxX, pos.x - baseX);
                
                // Check if projectile hit ground (Y position > groundTop)
                if (pos.y >= this.groundTop) {
                    tempProjectileHitGround = true;
                }
            }
        }
        
        this.stats.estimatedDistance = maxX.toFixed(2);
        console.log('🎯 Estimated Distance (simulated):', maxX.toFixed(2), 'm');
        
        // Update stats display if available
        if (window.updateStats) {
            window.updateStats(this.getStats());
        }
    }

    getStats() {
        return {
            distance: Math.max(0, this.stats.distance).toFixed(2),
            height: Math.max(0, this.stats.height).toFixed(2),
            velocity: Math.max(0, this.stats.velocity).toFixed(2),
            maxDistance: Math.max(0, this.stats.maxDistance).toFixed(2),
            maxHeight: Math.max(0, this.stats.maxHeight).toFixed(2),
            maxVelocity: Math.max(0, this.stats.maxVelocity).toFixed(2),
            time: this.stats.time.toFixed(2),
            estimatedDistance: this.stats.estimatedDistance || '0'
        };
    }
    
    setPlaySpeed(speed) {
        // Planck.js doesn't have built-in time scaling
        // Would need to adjust step size in animate()
        this.timeScale = parseFloat(speed);
    }
}
