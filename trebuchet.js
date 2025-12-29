// Trebuchet Simulator using Matter.js
// This module handles the physics simulation and trebuchet construction

const SCALE = 20; // pixels per meter

class TrebuchetSimulator {
    constructor(canvas) {
        this.canvas = canvas;
        this.setupPhysics();
        this.trebuchetType = 'hinged';
        this.parameters = this.getDefaultParameters();
        this.fired = false;
        this.paused = false;
        this.stats = { distance: 0, maxHeight: 0, time: 0 };
        this.projectileTrajectory = [];
    }

    setupPhysics() {
        // Create engine
        this.engine = Matter.Engine.create();
        this.world = this.engine.world;
        this.world.gravity.y = 1; // Matter.js default gravity

        // Create renderer
        const rect = this.canvas.getBoundingClientRect();
        this.render = Matter.Render.create({
            canvas: this.canvas,
            engine: this.engine,
            options: {
                width: rect.width,
                height: rect.height,
                wireframes: false,
                background: 'transparent'
            }
        });

        // Create ground
        const ground = Matter.Bodies.rectangle(
            rect.width / 2, 
            rect.height - 10, 
            rect.width, 
            20, 
            { 
                isStatic: true,
                render: {
                    fillStyle: '#654321'
                }
            }
        );
        Matter.World.add(this.world, ground);

        // Start renderer
        Matter.Render.run(this.render);

        // Create runner
        this.runner = Matter.Runner.create();
        Matter.Runner.run(this.runner, this.engine);

        // Add event listener for collision detection and tracking
        Matter.Events.on(this.engine, 'afterUpdate', () => {
            if (this.projectile && this.fired) {
                const pos = this.projectile.position;
                this.projectileTrajectory.push({ x: pos.x, y: pos.y });
                
                // Update stats
                const distance = (pos.x - this.startX) / SCALE;
                const height = (this.startY - pos.y) / SCALE;
                
                if (distance > this.stats.distance) {
                    this.stats.distance = distance;
                }
                if (height > this.stats.maxHeight) {
                    this.stats.maxHeight = height;
                }
                this.stats.time += 1/60; // Approximate time increment
            }
        });
    }

    getDefaultParameters() {
        return {
            armLength: 6,
            counterweightMass: 200,
            counterweightSize: 1,
            projectileMass: 10,
            projectileSize: 0.2,
            slingLength: 4,
            armMass: 30,
            releaseAngle: 45
        };
    }

    buildTrebuchet(type, params) {
        // Clear existing trebuchet
        if (this.trebuchetBodies) {
            this.trebuchetBodies.forEach(body => {
                Matter.World.remove(this.world, body);
            });
        }
        if (this.constraints) {
            this.constraints.forEach(constraint => {
                Matter.World.remove(this.world, constraint);
            });
        }

        this.trebuchetType = type;
        this.parameters = { ...this.getDefaultParameters(), ...params };
        this.trebuchetBodies = [];
        this.constraints = [];
        this.fired = false;
        this.projectileTrajectory = [];
        this.stats = { distance: 0, maxHeight: 0, time: 0 };

        const rect = this.canvas.getBoundingClientRect();
        const baseX = 200;
        const baseY = rect.height - 50;

        switch (type) {
            case 'hinged':
                this.buildHingedTrebuchet(baseX, baseY);
                break;
            case 'whipper':
                this.buildWhipperTrebuchet(baseX, baseY);
                break;
            case 'floating':
                this.buildFloatingArmTrebuchet(baseX, baseY);
                break;
            case 'walking':
                this.buildWalkingArmTrebuchet(baseX, baseY);
                break;
        }
    }

    buildHingedTrebuchet(baseX, baseY) {
        const p = this.parameters;
        const shortArmRatio = 0.3; // Short arm is 30% of total length
        const longArmRatio = 0.7;  // Long arm is 70% of total length

        // Base/Frame
        const frame = Matter.Bodies.rectangle(
            baseX, baseY, 20, 100,
            { 
                isStatic: true, 
                render: { fillStyle: '#8B4513' }
            }
        );
        this.trebuchetBodies.push(frame);

        // Throwing arm (pivots at the fulcrum)
        const armWidth = 15;
        const arm = Matter.Bodies.rectangle(
            baseX, baseY - 50, p.armLength * SCALE, armWidth,
            { 
                mass: p.armMass,
                render: { fillStyle: '#D2691E' }
            }
        );
        this.trebuchetBodies.push(arm);

        // Pivot constraint at fulcrum (about 1/3 from counterweight end)
        const pivotConstraint = Matter.Constraint.create({
            bodyA: frame,
            bodyB: arm,
            pointA: { x: 0, y: -50 },
            pointB: { x: -(p.armLength * longArmRatio - p.armLength / 2) * SCALE, y: 0 },
            length: 0,
            stiffness: 1
        });
        this.constraints.push(pivotConstraint);

        // Counterweight (hinged)
        const cwSize = p.counterweightSize * SCALE;
        const counterweight = Matter.Bodies.rectangle(
            baseX - (p.armLength * longArmRatio) * SCALE, 
            baseY - 50 - 20, 
            cwSize, cwSize,
            {
                mass: p.counterweightMass,
                render: { fillStyle: '#696969' }
            }
        );
        this.trebuchetBodies.push(counterweight);

        // Hinge for counterweight
        const cwHinge = Matter.Constraint.create({
            bodyA: arm,
            bodyB: counterweight,
            pointA: { x: -(p.armLength * longArmRatio) * SCALE, y: 0 },
            pointB: { x: 0, y: -cwSize / 2 },
            length: 20,
            stiffness: 0.8
        });
        this.constraints.push(cwHinge);

        // Projectile
        const projectileRadius = p.projectileSize * SCALE;
        this.projectile = Matter.Bodies.circle(
            baseX + (p.armLength * shortArmRatio) * SCALE,
            baseY - 50 - p.slingLength * SCALE,
            projectileRadius,
            {
                mass: p.projectileMass,
                render: { fillStyle: '#FF4500' },
                restitution: 0.6,
                friction: 0.5
            }
        );
        this.startX = this.projectile.position.x;
        this.startY = this.projectile.position.y;
        this.trebuchetBodies.push(this.projectile);

        // Sling (two ropes)
        const slingPoint = { x: (p.armLength * shortArmRatio) * SCALE, y: 0 };
        
        this.slingConstraint1 = Matter.Constraint.create({
            bodyA: arm,
            bodyB: this.projectile,
            pointA: slingPoint,
            length: p.slingLength * SCALE,
            stiffness: 0.9,
            render: { strokeStyle: '#8B4513', lineWidth: 2 }
        });
        
        this.slingConstraint2 = Matter.Constraint.create({
            bodyA: arm,
            bodyB: this.projectile,
            pointA: slingPoint,
            length: p.slingLength * SCALE,
            stiffness: 0.9,
            render: { strokeStyle: '#8B4513', lineWidth: 2 }
        });
        
        this.constraints.push(this.slingConstraint1, this.slingConstraint2);

        // Add all bodies and constraints to world
        Matter.World.add(this.world, this.trebuchetBodies);
        Matter.World.add(this.world, this.constraints);
    }

    buildWhipperTrebuchet(baseX, baseY) {
        const p = this.parameters;
        
        // Similar to hinged but with a second arm segment for whipping action
        const shortArmRatio = 0.3;
        const longArmRatio = 0.7;

        // Base/Frame
        const frame = Matter.Bodies.rectangle(
            baseX, baseY, 20, 100,
            { 
                isStatic: true, 
                render: { fillStyle: '#8B4513' }
            }
        );
        this.trebuchetBodies.push(frame);

        // Main throwing arm
        const armWidth = 15;
        const mainArm = Matter.Bodies.rectangle(
            baseX, baseY - 50, p.armLength * SCALE, armWidth,
            { 
                mass: p.armMass * 0.7,
                render: { fillStyle: '#D2691E' }
            }
        );
        this.trebuchetBodies.push(mainArm);

        // Pivot constraint
        const pivotConstraint = Matter.Constraint.create({
            bodyA: frame,
            bodyB: mainArm,
            pointA: { x: 0, y: -50 },
            pointB: { x: -(p.armLength * longArmRatio - p.armLength / 2) * SCALE, y: 0 },
            length: 0,
            stiffness: 1
        });
        this.constraints.push(pivotConstraint);

        // Counterweight
        const cwSize = p.counterweightSize * SCALE;
        const counterweight = Matter.Bodies.rectangle(
            baseX - (p.armLength * longArmRatio) * SCALE, 
            baseY - 50 - 20, 
            cwSize, cwSize,
            {
                mass: p.counterweightMass,
                render: { fillStyle: '#696969' }
            }
        );
        this.trebuchetBodies.push(counterweight);

        const cwHinge = Matter.Constraint.create({
            bodyA: mainArm,
            bodyB: counterweight,
            pointA: { x: -(p.armLength * longArmRatio) * SCALE, y: 0 },
            pointB: { x: 0, y: -cwSize / 2 },
            length: 20,
            stiffness: 0.8
        });
        this.constraints.push(cwHinge);

        // Whipper arm (extends from main arm)
        const whipperLength = p.armLength * 0.4;
        const whipperArm = Matter.Bodies.rectangle(
            baseX + (p.armLength * shortArmRatio) * SCALE + whipperLength * SCALE / 2,
            baseY - 50,
            whipperLength * SCALE, 12,
            {
                mass: p.armMass * 0.3,
                render: { fillStyle: '#CD853F' }
            }
        );
        this.trebuchetBodies.push(whipperArm);

        // Joint between main arm and whipper
        const whipperJoint = Matter.Constraint.create({
            bodyA: mainArm,
            bodyB: whipperArm,
            pointA: { x: (p.armLength * shortArmRatio) * SCALE, y: 0 },
            pointB: { x: -whipperLength * SCALE / 2, y: 0 },
            length: 0,
            stiffness: 0.3 // Flexible joint for whipping action
        });
        this.constraints.push(whipperJoint);

        // Projectile
        const projectileRadius = p.projectileSize * SCALE;
        this.projectile = Matter.Bodies.circle(
            baseX + (p.armLength * shortArmRatio + whipperLength) * SCALE,
            baseY - 50 - p.slingLength * SCALE,
            projectileRadius,
            {
                mass: p.projectileMass,
                render: { fillStyle: '#FF4500' },
                restitution: 0.6,
                friction: 0.5
            }
        );
        this.startX = this.projectile.position.x;
        this.startY = this.projectile.position.y;
        this.trebuchetBodies.push(this.projectile);

        // Sling attached to whipper arm
        const slingPoint = { x: whipperLength * SCALE / 2, y: 0 };
        
        this.slingConstraint1 = Matter.Constraint.create({
            bodyA: whipperArm,
            bodyB: this.projectile,
            pointA: slingPoint,
            length: p.slingLength * SCALE,
            stiffness: 0.9,
            render: { strokeStyle: '#8B4513', lineWidth: 2 }
        });
        
        this.slingConstraint2 = Matter.Constraint.create({
            bodyA: whipperArm,
            bodyB: this.projectile,
            pointA: slingPoint,
            length: p.slingLength * SCALE,
            stiffness: 0.9,
            render: { strokeStyle: '#8B4513', lineWidth: 2 }
        });
        
        this.constraints.push(this.slingConstraint1, this.slingConstraint2);

        // Add all bodies and constraints to world
        Matter.World.add(this.world, this.trebuchetBodies);
        Matter.World.add(this.world, this.constraints);
    }

    buildFloatingArmTrebuchet(baseX, baseY) {
        const p = this.parameters;
        
        // Floating arm design - the arm pivot can move horizontally
        const shortArmRatio = 0.3;
        const longArmRatio = 0.7;

        // Base track (static)
        const track = Matter.Bodies.rectangle(
            baseX, baseY, 150, 20,
            { 
                isStatic: true, 
                render: { fillStyle: '#8B4513' }
            }
        );
        this.trebuchetBodies.push(track);

        // Movable pivot/frame that can slide on track
        const floatingFrame = Matter.Bodies.rectangle(
            baseX, baseY - 30, 20, 60,
            { 
                mass: 50,
                friction: 0.1,
                render: { fillStyle: '#A0522D' }
            }
        );
        this.trebuchetBodies.push(floatingFrame);

        // Constraint to keep frame on track (vertical movement restricted)
        const trackConstraint = Matter.Constraint.create({
            bodyA: track,
            bodyB: floatingFrame,
            pointA: { x: 0, y: -10 },
            pointB: { x: 0, y: 30 },
            length: 0,
            stiffness: 1
        });
        this.constraints.push(trackConstraint);

        // Throwing arm
        const armWidth = 15;
        const arm = Matter.Bodies.rectangle(
            baseX, baseY - 60, p.armLength * SCALE, armWidth,
            { 
                mass: p.armMass,
                render: { fillStyle: '#D2691E' }
            }
        );
        this.trebuchetBodies.push(arm);

        // Pivot on floating frame
        const pivotConstraint = Matter.Constraint.create({
            bodyA: floatingFrame,
            bodyB: arm,
            pointA: { x: 0, y: -30 },
            pointB: { x: -(p.armLength * longArmRatio - p.armLength / 2) * SCALE, y: 0 },
            length: 0,
            stiffness: 1
        });
        this.constraints.push(pivotConstraint);

        // Counterweight
        const cwSize = p.counterweightSize * SCALE;
        const counterweight = Matter.Bodies.rectangle(
            baseX - (p.armLength * longArmRatio) * SCALE, 
            baseY - 60 - 20, 
            cwSize, cwSize,
            {
                mass: p.counterweightMass,
                render: { fillStyle: '#696969' }
            }
        );
        this.trebuchetBodies.push(counterweight);

        const cwHinge = Matter.Constraint.create({
            bodyA: arm,
            bodyB: counterweight,
            pointA: { x: -(p.armLength * longArmRatio) * SCALE, y: 0 },
            pointB: { x: 0, y: -cwSize / 2 },
            length: 20,
            stiffness: 0.8
        });
        this.constraints.push(cwHinge);

        // Projectile
        const projectileRadius = p.projectileSize * SCALE;
        this.projectile = Matter.Bodies.circle(
            baseX + (p.armLength * shortArmRatio) * SCALE,
            baseY - 60 - p.slingLength * SCALE,
            projectileRadius,
            {
                mass: p.projectileMass,
                render: { fillStyle: '#FF4500' },
                restitution: 0.6,
                friction: 0.5
            }
        );
        this.startX = this.projectile.position.x;
        this.startY = this.projectile.position.y;
        this.trebuchetBodies.push(this.projectile);

        // Sling
        const slingPoint = { x: (p.armLength * shortArmRatio) * SCALE, y: 0 };
        
        this.slingConstraint1 = Matter.Constraint.create({
            bodyA: arm,
            bodyB: this.projectile,
            pointA: slingPoint,
            length: p.slingLength * SCALE,
            stiffness: 0.9,
            render: { strokeStyle: '#8B4513', lineWidth: 2 }
        });
        
        this.slingConstraint2 = Matter.Constraint.create({
            bodyA: arm,
            bodyB: this.projectile,
            pointA: slingPoint,
            length: p.slingLength * SCALE,
            stiffness: 0.9,
            render: { strokeStyle: '#8B4513', lineWidth: 2 }
        });
        
        this.constraints.push(this.slingConstraint1, this.slingConstraint2);

        // Add all bodies and constraints to world
        Matter.World.add(this.world, this.trebuchetBodies);
        Matter.World.add(this.world, this.constraints);
    }

    buildWalkingArmTrebuchet(baseX, baseY) {
        const p = this.parameters;
        
        // Walking arm design - the base can pivot/walk
        const shortArmRatio = 0.3;
        const longArmRatio = 0.7;

        // Walking base (can rock back and forth)
        const baseWidth = 120;
        const walkingBase = Matter.Bodies.trapezoid(
            baseX, baseY, baseWidth, 40, 0.5,
            { 
                mass: 100,
                friction: 1,
                render: { fillStyle: '#654321' }
            }
        );
        this.trebuchetBodies.push(walkingBase);

        // Frame attached to base
        const frame = Matter.Bodies.rectangle(
            baseX, baseY - 50, 20, 80,
            { 
                mass: 30,
                render: { fillStyle: '#8B4513' }
            }
        );
        this.trebuchetBodies.push(frame);

        // Attach frame to base
        const frameConstraint = Matter.Constraint.create({
            bodyA: walkingBase,
            bodyB: frame,
            pointA: { x: 0, y: -20 },
            pointB: { x: 0, y: 40 },
            length: 0,
            stiffness: 0.9
        });
        this.constraints.push(frameConstraint);

        // Throwing arm
        const armWidth = 15;
        const arm = Matter.Bodies.rectangle(
            baseX, baseY - 80, p.armLength * SCALE, armWidth,
            { 
                mass: p.armMass,
                render: { fillStyle: '#D2691E' }
            }
        );
        this.trebuchetBodies.push(arm);

        // Pivot constraint
        const pivotConstraint = Matter.Constraint.create({
            bodyA: frame,
            bodyB: arm,
            pointA: { x: 0, y: -40 },
            pointB: { x: -(p.armLength * longArmRatio - p.armLength / 2) * SCALE, y: 0 },
            length: 0,
            stiffness: 1
        });
        this.constraints.push(pivotConstraint);

        // Counterweight
        const cwSize = p.counterweightSize * SCALE;
        const counterweight = Matter.Bodies.rectangle(
            baseX - (p.armLength * longArmRatio) * SCALE, 
            baseY - 80 - 20, 
            cwSize, cwSize,
            {
                mass: p.counterweightMass,
                render: { fillStyle: '#696969' }
            }
        );
        this.trebuchetBodies.push(counterweight);

        const cwHinge = Matter.Constraint.create({
            bodyA: arm,
            bodyB: counterweight,
            pointA: { x: -(p.armLength * longArmRatio) * SCALE, y: 0 },
            pointB: { x: 0, y: -cwSize / 2 },
            length: 20,
            stiffness: 0.8
        });
        this.constraints.push(cwHinge);

        // Projectile
        const projectileRadius = p.projectileSize * SCALE;
        this.projectile = Matter.Bodies.circle(
            baseX + (p.armLength * shortArmRatio) * SCALE,
            baseY - 80 - p.slingLength * SCALE,
            projectileRadius,
            {
                mass: p.projectileMass,
                render: { fillStyle: '#FF4500' },
                restitution: 0.6,
                friction: 0.5
            }
        );
        this.startX = this.projectile.position.x;
        this.startY = this.projectile.position.y;
        this.trebuchetBodies.push(this.projectile);

        // Sling
        const slingPoint = { x: (p.armLength * shortArmRatio) * SCALE, y: 0 };
        
        this.slingConstraint1 = Matter.Constraint.create({
            bodyA: arm,
            bodyB: this.projectile,
            pointA: slingPoint,
            length: p.slingLength * SCALE,
            stiffness: 0.9,
            render: { strokeStyle: '#8B4513', lineWidth: 2 }
        });
        
        this.slingConstraint2 = Matter.Constraint.create({
            bodyA: arm,
            bodyB: this.projectile,
            pointA: slingPoint,
            length: p.slingLength * SCALE,
            stiffness: 0.9,
            render: { strokeStyle: '#8B4513', lineWidth: 2 }
        });
        
        this.constraints.push(this.slingConstraint1, this.slingConstraint2);

        // Add all bodies and constraints to world
        Matter.World.add(this.world, this.trebuchetBodies);
        Matter.World.add(this.world, this.constraints);
    }

    fire() {
        if (this.fired || !this.projectile) return;
        
        this.fired = true;
        this.stats = { distance: 0, maxHeight: 0, time: 0 };
        
        // Release sling at appropriate angle
        const releaseAngleRad = (this.parameters.releaseAngle * Math.PI) / 180;
        
        setTimeout(() => {
            if (this.slingConstraint1 && this.slingConstraint2) {
                Matter.World.remove(this.world, this.slingConstraint1);
                Matter.World.remove(this.world, this.slingConstraint2);
            }
        }, 100); // Small delay for better release timing
    }

    reset() {
        this.buildTrebuchet(this.trebuchetType, this.parameters);
    }

    pause() {
        this.paused = !this.paused;
        if (this.paused) {
            Matter.Runner.stop(this.runner);
        } else {
            Matter.Runner.run(this.runner, this.engine);
        }
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
}
