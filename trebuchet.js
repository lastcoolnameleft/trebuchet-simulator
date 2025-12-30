// Trebuchet Simulator using Planck.js
// Main simulator class that coordinates physics and trebuchet construction

const SCALE = 20; // pixels per meter

class TrebuchetSimulator {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.setupPhysics();
        this.trebuchetType = 'sandbox';
        this.parameters = this.getDefaultParameters();
        this.fired = false;
        this.paused = true; // Start paused
        this.stats = { distance: 0, maxHeight: 0, time: 0 };
        this.projectileTrajectory = [];
        this.animationId = null;
        this.projectileHitGround = false; // Track if projectile hit ground
        // Initialize trebuchet builders
        this.builders = {
            hinged: new HingedTrebuchetBuilder(this),
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
            
            // Check if projectile hit ground
            if ((bodyA === this.projectile && bodyB === this.ground) ||
                (bodyB === this.projectile && bodyA === this.ground)) {
                if (!this.projectileHitGround) {
                    this.projectileHitGround = true;
                    // Stop the projectile
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

        // Create ground (thin ground layer extended to the left)
        const groundHalfHeight = 0.5; // Thinner ground (was 2.5)
        const groundHalfWidth = rect.width / SCALE; // Double width to extend left
        const groundCenterX = rect.width / 2 / SCALE; // Keep center at canvas middle
        const groundBody = this.world.createBody({
            position: planck.Vec2(groundCenterX, (rect.height - groundHalfHeight) / SCALE)
        });
        groundBody.createFixture({
            shape: planck.Box(groundHalfWidth, groundHalfHeight),
            friction: 0.5,
            userData: { color: '#654321' }
        });
        this.ground = groundBody;
        // Calculate ground top surface: ground center Y minus half-height
        this.groundTop = (rect.height - groundHalfHeight) / SCALE - groundHalfHeight;

        // Start animation loop
        this.animate();
    }

    animate() {
        if (!this.paused) {
            this.world.step(1/60);
            
            // Check for sling release (when arm rotates clockwise past 1.5 radians, from planck.html)
            if (this.slingJoint && this.trebuchetBodies && this.trebuchetBodies[1]) {
                const arm = this.trebuchetBodies[1]; // Arm is second body
                if (arm.getAngle() > 1.5) {
                    this.world.destroyJoint(this.slingJoint);
                    this.slingJoint = null;
                    console.log('Sling released at angle:', arm.getAngle());
                }
            }
            
            // Track projectile
            if (this.projectile) {
                const pos = this.projectile.getPosition();
                this.projectileTrajectory.push({ x: pos.x * SCALE, y: pos.y * SCALE });
                
                // Update stats - real-time accurate
                // Distance from starting X position (in meters)
                const distance = pos.x - this.startX;
                
                // Height above starting position (Y increases downward, so subtract)
                const height = this.startY - pos.y;
                
                // Update max distance (always show current distance if projectile moved right)
                this.stats.distance = Math.max(0, distance);
                
                // Update max height reached
                if (height > this.stats.maxHeight) {
                    this.stats.maxHeight = height;
                }
                
                // Track time (only if projectile hasn't hit ground yet)
                if (!this.projectileHitGround) {
                    this.stats.time += 1/60;
                }
            }
        }
        
        this.render();
        this.animationId = requestAnimationFrame(() => this.animate());
    }

    render() {
        const rect = this.canvas.getBoundingClientRect();
        this.ctx.fillStyle = '#0f0f13';
        this.ctx.fillRect(0, 0, rect.width, rect.height);
        
        // Camera offset - shift view to the left
        const cameraOffsetX = 200; // Negative value moves camera left (scene appears to move right)
        
        // Draw all bodies
        for (let body = this.world.getBodyList(); body; body = body.getNext()) {
            this.ctx.save();
            const pos = body.getPosition();
            this.ctx.translate(pos.x * SCALE + cameraOffsetX, pos.y * SCALE);
            this.ctx.rotate(body.getAngle());
            
            for (let fixture = body.getFixtureList(); fixture; fixture = fixture.getNext()) {
                const shape = fixture.getShape();
                
                if (shape.getType() === 'circle') {
                    this.ctx.fillStyle = fixture.getUserData()?.color || '#8B4513';
                    this.ctx.beginPath();
                    this.ctx.arc(0, 0, shape.getRadius() * SCALE, 0, Math.PI * 2);
                    this.ctx.fill();
                } else if (shape.getType() === 'polygon') {
                    const vertices = shape.m_vertices;
                    this.ctx.fillStyle = fixture.getUserData()?.color || '#654321';
                    this.ctx.beginPath();
                    this.ctx.moveTo(vertices[0].x * SCALE, vertices[0].y * SCALE);
                    for (let i = 1; i < vertices.length; i++) {
                        this.ctx.lineTo(vertices[i].x * SCALE, vertices[i].y * SCALE);
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
            this.ctx.moveTo(anchorA.x * SCALE + cameraOffsetX, anchorA.y * SCALE);
            this.ctx.lineTo(anchorB.x * SCALE + cameraOffsetX, anchorB.y * SCALE);
            this.ctx.stroke();
        }
    }

    getDefaultParameters() {
        return {
            armLength: 20,
            counterweightMass: 200,
            counterweightSize: 1,
            projectileMass: 10,
            projectileSize: 0.2,
            slingLength: 4,
            armMass: 30,
            releaseAngle: 45,
            armHeight: 13
        };
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
        this.stats = { distance: 0, maxHeight: 0, time: 0 };

        const rect = this.canvas.getBoundingClientRect();
        const baseX = 200 / SCALE;
        const baseY = this.groundTop; // Use stored ground top position

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
        this.stats = { distance: 0, maxHeight: 0, time: 0 };
        
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
    }

    pause() {
        this.paused = !this.paused;
        return this.paused;
    }

    updateParameter(param, value) {
        this.parameters[param] = parseFloat(value);
        this.reset();
    }

    getStats() {
        return {
            distance: Math.max(0, this.stats.distance).toFixed(2),
            maxHeight: Math.max(0, this.stats.maxHeight).toFixed(2),
            time: this.stats.time.toFixed(2)
        };
    }
    
    setPlaySpeed(speed) {
        // Planck.js doesn't have built-in time scaling
        // Would need to adjust step size in animate()
        this.timeScale = parseFloat(speed);
    }
}
