// Hinged Trebuchet
// Based on planck.html working implementation
// Customizable starting point for experimentation

class FixedCounterweightTrebuchetBuilder extends BaseTrebuchetBuilder {
  static getParameterConfig() {
    return [
      { id: 'projectileArmLength', label: 'Arm Length (Projectile)', unit: 'm', step: 1, min: 1, max: 50, default: 14 },
      { id: 'counterweightArmLength', label: 'Arm Length (Counterweight)', unit: 'm', step: 1, min: 1, max: 30, default: 6 },
      { id: 'armMass', label: 'Arm Mass', unit: 'kg', step: 5, min: 5, max: 500, default: 30 },
      { id: 'counterweightMass', label: 'Counterweight Mass', unit: 'kg', step: 10, min: 10, max: 1000, default: 200 },
      { id: 'counterweightSize', label: 'Counterweight Size', unit: 'm', step: 0.1, min: 0.1, max: 5, default: 1 },
      { id: 'projectileMass', label: 'Projectile Mass', unit: 'kg', step: 1, min: 1, max: 100, default: 10 },
      { id: 'projectileSize', label: 'Projectile Diameter', unit: 'm', step: 0.05, min: 0.1, max: 3, default: 0.4 },
      { id: 'slingLength', label: 'Sling Length', unit: 'm', step: 0.5, min: 1, max: 30, default: 12 },
      { id: 'releaseAngle', label: 'Release Angle', unit: '°', step: 5, min: 0, max: 90, default: 45 }
    ];
  }
  build(baseX, baseY, params) {
    const bodies = [];
    const joints = [];
    const leftArmLength = params.projectileArmLength;
    const rightArmLength = params.counterweightArmLength;
    const armCenterOffset = (rightArmLength - leftArmLength) / 2;

    // Vertical frame - height adjusts with arm height
    const armHeight = params.armHeight || 22.5; // Customizable arm height (default: top of frame)
    const frameHeight = armHeight / 2; // Half-height for Box shape
    const frameY = baseY - frameHeight; // Center frame between ground and pivot
    const frame = this.simulator.world.createBody({
      position: planck.Vec2(baseX, frameY),
      userData: { name: "frame" }, // Debug label
    });
    frame.createFixture({
      shape: planck.Box(0.5, frameHeight),
      friction: 0.5,
      filterCategoryBits: 0x0002,
      filterMaskBits: 0x0001,
      userData: { color: "#8B4513", name: "frame" },
    });
    bodies.push(frame);

    // Catapult arm - 70/30 split with offset center
    const pivotY = baseY - armHeight;
    const arm = this.simulator.world.createBody({
      position: planck.Vec2(baseX, pivotY),
      type: "dynamic",
      angle: -Math.PI / 4, // 45 degrees (from planck.html)
      userData: { name: "arm" }, // Debug label
    });
    arm.createFixture({
      shape: planck.Box(
        (leftArmLength + rightArmLength) / 2,
        1 / 2,
        planck.Vec2(armCenterOffset, 0),
        0
      ),
      density: 1.0,
      friction: 0.5,
      filterCategoryBits: 0x0010, // Arm category
      filterMaskBits: 0x0001, // Only collide with ground
      userData: { color: "#D2691E", name: "arm" },
    });
    bodies.push(arm);

    // Pivot joint (from planck.html)
    const pivotJoint = this.simulator.world.createJoint(
      planck.RevoluteJoint({
        bodyA: frame,
        bodyB: arm,
        localAnchorA: planck.Vec2(0, pivotY - frameY), // Relative to frame center
        localAnchorB: planck.Vec2(0, 0),
        enableLimit: false,
      })
    );
    joints.push(pivotJoint);

    // Counterweight - directly attached to arm (fixed, not hanging)
    const cwRadius = params.counterweightSize || 2.5;
    const armAngle = -Math.PI / 4; // Same angle as arm

    // Calculate the world position of the counterweight attachment point on the arm
    const cwAttachX = baseX + rightArmLength * Math.cos(armAngle);
    const cwAttachY = pivotY + rightArmLength * Math.sin(armAngle);

    // Position counterweight directly at attachment point (fixed to arm)
    const cwX = cwAttachX;
    const cwY = cwAttachY;

    const counterweight = this.simulator.world.createBody({
      position: planck.Vec2(cwX, cwY),
      type: "dynamic",
      linearDamping: 0.1,
      angularDamping: 0.1,
      userData: { name: "counterweight" }, // Debug label
    });
    // Calculate density from desired mass: mass = density * area
    // For circle: area = π * r², so density = mass / (π * r²)
    const cwArea = Math.PI * cwRadius * cwRadius;
    const cwDensity = (params.counterweightMass || 200) / cwArea;
    counterweight.createFixture({
      shape: planck.Circle(cwRadius),
      density: cwDensity,
      friction: 0.5,
      filterCategoryBits: 0x0004,
      filterMaskBits: 0x0000, // Don't collide with anything!
      userData: { color: "#696969", name: "counterweight" },
    });
    // Ensure counterweight starts at rest
    counterweight.setLinearVelocity(planck.Vec2(0, 0));
    counterweight.setAngularVelocity(0);
    bodies.push(counterweight);

    // Fixed counterweight - weld joint to make it rigidly attached to arm
    const cwWeld = this.simulator.world.createJoint(
      planck.WeldJoint({
        bodyA: arm,
        bodyB: counterweight,
        localAnchorA: planck.Vec2(rightArmLength, 0),
        localAnchorB: planck.Vec2(0, 0),
        referenceAngle: 0,
        frequencyHz: 0.0,  // Rigid connection
        dampingRatio: 0.0,
      })
    );
    joints.push(cwWeld);

    // Projectile - calculate position based on arm angle and sling length
    const projRadius = 0.75;
    const slingLength = params.slingLength || 6;

    // Calculate the world position of the sling attachment point on the arm (accounting for rotation)
    // Reuse armAngle from above
    // The attachment is at -leftArmLength along the arm (in local coordinates)
    // Convert to world coordinates considering the arm's rotation
    const slingAttachX = baseX + -leftArmLength * Math.cos(armAngle);
    const slingAttachY = pivotY + -leftArmLength * Math.sin(armAngle);

    // Position projectile on ground with center at ground level + radius (so bottom touches ground)
    const projY = baseY - projRadius;
    const verticalDistance = Math.abs(projY - slingAttachY);

    // Calculate horizontal position to maintain exact sling length
    let projX;
    if (slingLength > verticalDistance) {
      const horizontalSlack = Math.sqrt(
        slingLength * slingLength - verticalDistance * verticalDistance
      );
      projX = slingAttachX + horizontalSlack; // Extend to the right
    } else {
      projX = slingAttachX; // Directly below if sling is shorter
    }

    // Verify distance (for debugging)
    const actualDistance = Math.sqrt(
      Math.pow(projX - slingAttachX, 2) + Math.pow(projY - slingAttachY, 2)
    );
    console.log(
      `Sling length: ${slingLength}, Actual distance: ${actualDistance.toFixed(
        3
      )}`
    );

    const projectile = this.simulator.world.createBody({
      position: planck.Vec2(projX, projY),
      type: "dynamic",
      linearDamping: 0.1,
      angularDamping: 0.1,
      bullet: true, // Enable CCD for fast-moving projectile
      userData: { name: "projectile" }, // Debug label
    });
    projectile.createFixture({
      shape: planck.Circle(projRadius),
      density: 1.0,
      friction: 10.0, // High friction to stop quickly on ground
      restitution: 0.0, // No bounce
      filterCategoryBits: 0x0008,
      filterMaskBits: 0x0001, // Only collide with ground
      userData: { color: "#FF4500", name: "projectile" },
    });
    // Ensure projectile starts at rest
    projectile.setLinearVelocity(planck.Vec2(0, 0));
    projectile.setAngularVelocity(0);
    bodies.push(projectile);

    // Track previous velocity for debugging sudden changes
    let lastVelocity = planck.Vec2(0, 0);
    let lastPosition = planck.Vec2(projX, projY);
    let velocityChangeCount = 0;

    // Track projectile state every frame
    let frameCount = 0;
    let projectileReleased = false;

    // Add pre-step listener to track every frame
    this.simulator.world.on("pre-step", () => {
      // FIRST: Enforce stopped state if projectile hit ground
      if (this.simulator.projectileHitGround) {
        projectile.setLinearVelocity(planck.Vec2(0, 0));
        projectile.setAngularVelocity(0);
        if (projectile.getType() !== "static") {
          projectile.setType("static");
        }
        return; // Skip all other logging/tracking
      }

      const currentVel = projectile.getLinearVelocity();
      const currentPos = projectile.getPosition();

      // Check if projectile has been released (significant velocity)
      if (
        !projectileReleased &&
        (Math.abs(currentVel.x) > 5 || Math.abs(currentVel.y) > 5)
      ) {
        projectileReleased = true;
        console.log(
          "🚀 PROJECTILE RELEASED at position:",
          currentPos,
          "velocity:",
          currentVel
        );
      }

      // Calculate direction change (angle between velocity vectors)
      const lastSpeed = Math.sqrt(
        lastVelocity.x * lastVelocity.x + lastVelocity.y * lastVelocity.y
      );
      const currentSpeed = Math.sqrt(
        currentVel.x * currentVel.x + currentVel.y * currentVel.y
      );
      let directionChange = 0;
      if (lastSpeed > 1 && currentSpeed > 1) {
        // Dot product to find angle between vectors
        const dotProduct =
          (lastVelocity.x * currentVel.x + lastVelocity.y * currentVel.y) /
          (lastSpeed * currentSpeed);
        directionChange =
          (Math.acos(Math.max(-1, Math.min(1, dotProduct))) * 180) / Math.PI;
      }

      // Check if sling still exists and is affecting projectile
      //            if (projectileReleased && slingJoint && frameCount < 600) {
      //                console.warn(`⚠️ Frame ${frameCount}: SLING JOINT STILL EXISTS! This should not happen after release!`);
      //            }

      // Log EVERY frame if projectile is released
      if (projectileReleased && frameCount < 600) {
        // Stop at frame 600 to avoid too many logs
        // Detect apex (Y velocity crossing zero in Y+ down system)
        const wasRising = lastVelocity.y < -0.5; // Negative Y = rising (away from ground)
        const nowFalling = currentVel.y > 0.5; // Positive Y = falling (toward ground)

        const flightPhase =
          currentVel.y < -0.5
            ? "⬆️ RISING"
            : currentVel.y > 0.5
            ? "⬇️ FALLING"
            : "🔝 APEX";

        // Check for proximity to arm
        const armPos = arm.getPosition();
        const distanceToArm = Math.sqrt(
          Math.pow(currentPos.x - armPos.x, 2) +
            Math.pow(currentPos.y - armPos.y, 2)
        );
        const armWarning =
          distanceToArm < 15
            ? `⚠️ ARM NEARBY (${distanceToArm.toFixed(1)}m)`
            : "";

        // Detect sudden direction changes
        const velAngleChange =
          (Math.abs(
            Math.atan2(currentVel.y, currentVel.x) -
              Math.atan2(lastVelocity.y, lastVelocity.x)
          ) *
            180) /
          Math.PI;
        const suddenChange =
          velAngleChange > 10
            ? `🚨 SUDDEN ANGLE CHANGE: ${velAngleChange.toFixed(1)}°`
            : "";

        //console.log(`Frame ${frameCount}: Pos (${currentPos.x.toFixed(2)}, ${currentPos.y.toFixed(2)}) Vel (${currentVel.x.toFixed(2)}, ${currentVel.y.toFixed(2)}) ${flightPhase} Speed: ${currentSpeed.toFixed(2)} m/s ${suddenChange} ${armWarning}`);
      }

      lastVelocity = planck.Vec2(currentVel.x, currentVel.y);
      lastPosition = planck.Vec2(currentPos.x, currentPos.y);
      frameCount++;
    });

    // Add collision debugging
    this.simulator.world.on("begin-contact", (contact) => {
      const fixtureA = contact.getFixtureA();
      const fixtureB = contact.getFixtureB();
      const bodyA = fixtureA.getBody();
      const bodyB = fixtureB.getBody();
      const userDataA = bodyA.getUserData();
      const userDataB = bodyB.getUserData();
      const fixtureDataA = fixtureA.getUserData();
      const fixtureDataB = fixtureB.getUserData();

      // Check if projectile is involved
      if (
        (userDataA && userDataA.name === "projectile") ||
        (userDataB && userDataB.name === "projectile") ||
        (fixtureDataA && fixtureDataA.name === "projectile") ||
        (fixtureDataB && fixtureDataB.name === "projectile")
      ) {
        const projBody = bodyA === projectile ? bodyA : bodyB;
        const otherBody = bodyA === projectile ? bodyB : bodyA;
        const otherFixture = bodyA === projectile ? fixtureB : fixtureA;
        const otherUserData = bodyA === projectile ? userDataB : userDataA;
        const otherFixtureData =
          bodyA === projectile ? fixtureDataB : fixtureDataA;

        console.log("🔴 PROJECTILE COLLISION DETECTED:");
        console.log(
          "  Colliding with:",
          otherUserData?.name || otherFixtureData?.name || "UNKNOWN"
        );
        console.log("  Other body position:", otherBody.getPosition());
        console.log("  Projectile position:", projBody.getPosition());
        console.log(
          "  Projectile velocity BEFORE:",
          projBody.getLinearVelocity()
        );
        console.log(
          "  Other fixture category:",
          otherFixture.getFilterCategoryBits(),
          "mask:",
          otherFixture.getFilterMaskBits()
        );
        console.log(
          "  Projectile fixture category:",
          fixtureB.getFilterCategoryBits(),
          "mask:",
          fixtureB.getFilterMaskBits()
        );
        console.log("  Contact manifold:", contact.getManifold());
        console.log("  Is touching:", contact.isTouching());
        console.log("---");
      }
    });

    // Track velocity changes in post-solve (only for collision detection)
    this.simulator.world.on("post-solve", (contact, impulse) => {
      // FIRST: Check if projectile hit ground and stop it immediately AFTER physics solve
      const fixtureA = contact.getFixtureA();
      const fixtureB = contact.getFixtureB();
      const bodyA = fixtureA ? fixtureA.getBody() : null;
      const bodyB = fixtureB ? fixtureB.getBody() : null;

      // Debug: log if projectile is involved at all
      if (bodyA === projectile || bodyB === projectile) {
        const otherBody = bodyA === projectile ? bodyB : bodyA;
        const otherUserData = otherBody?.getUserData();
        const otherFixture = bodyA === projectile ? fixtureB : fixtureA;
        const otherFixtureData = otherFixture?.getUserData();
        const otherName =
          otherUserData?.name || otherFixtureData?.name || "UNKNOWN";

        //                console.log('📊 POST-SOLVE: projectile collision with', otherName,
        //                    'fired:', this.simulator.fired,
        //                    'hitGround:', this.simulator.projectileHitGround);

        // Check if hit ground
        if (
          (otherUserData && otherUserData.name === "ground") ||
          (otherFixtureData && otherFixtureData.name === "ground")
        ) {
          if (this.simulator.fired && !this.simulator.projectileHitGround) {
            console.log(
              "🎯 PROJECTILE HIT GROUND - STOPPING COMPLETELY (post-solve)"
            );
            projectile.setLinearVelocity(planck.Vec2(0, 0));
            projectile.setAngularVelocity(0);
            projectile.setType("static");
            this.simulator.projectileHitGround = true;
            return; // Skip velocity logging
            //                    } else {
            //                        console.log('⚠️ Ground collision but NOT stopping - fired:', this.simulator.fired, 'hitGround:', this.simulator.projectileHitGround);
          }
        }
      }

      const currentVel = projectile.getLinearVelocity();
      const currentPos = projectile.getPosition();

      // Calculate velocity change magnitude
      const velChange = Math.sqrt(
        Math.pow(currentVel.x - lastVelocity.x, 2) +
          Math.pow(currentVel.y - lastVelocity.y, 2)
      );

      // Only log significant velocity changes during collisions
      if (velChange > 1 && projectileReleased) {
        velocityChangeCount++;
        console.log(`⚡ VELOCITY CHANGE #${velocityChangeCount}:`);
        console.log("  Frame:", frameCount);
        console.log("  Position:", currentPos);
        console.log("  Old velocity:", lastVelocity);
        console.log("  New velocity:", currentVel);
        console.log("  Change magnitude:", velChange.toFixed(2), "m/s");
        console.log(
          "  Velocity X change:",
          (currentVel.x - lastVelocity.x).toFixed(2),
          "m/s"
        );
        console.log(
          "  Velocity Y change:",
          (currentVel.y - lastVelocity.y).toFixed(2),
          "m/s"
        );
        console.log("  Impulse:", impulse ? impulse.normalImpulses : "none");

        // Check what the contact was
        const fixtureA = contact.getFixtureA();
        const fixtureB = contact.getFixtureB();
        const bodyA = fixtureA ? fixtureA.getBody() : null;
        const bodyB = fixtureB ? fixtureB.getBody() : null;

        console.log("  Contact fixtures exist:", !!fixtureA, !!fixtureB);
        console.log(
          "  Projectile involved in contact:",
          bodyA === projectile,
          bodyB === projectile
        );

        if (bodyA && bodyB) {
          const userDataA = bodyA.getUserData();
          const userDataB = bodyB.getUserData();
          const fixtureDataA = fixtureA.getUserData();
          const fixtureDataB = fixtureB.getUserData();

          console.log(
            "  Body A:",
            userDataA?.name || fixtureDataA?.name || "UNKNOWN",
            "at",
            bodyA.getPosition()
          );
          console.log(
            "  Body B:",
            userDataB?.name || fixtureDataB?.name || "UNKNOWN",
            "at",
            bodyB.getPosition()
          );
          console.log(
            "  Body A filter - category:",
            fixtureA.getFilterCategoryBits().toString(16),
            "mask:",
            fixtureA.getFilterMaskBits().toString(16)
          );
          console.log(
            "  Body B filter - category:",
            fixtureB.getFilterCategoryBits().toString(16),
            "mask:",
            fixtureB.getFilterMaskBits().toString(16)
          );

          if (bodyA === projectile || bodyB === projectile) {
            const otherBody = bodyA === projectile ? bodyB : bodyA;
            const otherUserData = otherBody.getUserData();
            const otherFixture = bodyA === projectile ? fixtureB : fixtureA;
            const otherFixtureData = otherFixture.getUserData();
            const otherPosition = otherBody.getPosition();
            const otherVelocity = otherBody.getLinearVelocity();

            console.log(
              "  🎯 PROJECTILE COLLISION PARTNER:",
              otherUserData?.name || otherFixtureData?.name || "UNKNOWN"
            );
            console.log("  Partner velocity:", otherVelocity);
            console.log(
              "  Partner angle:",
              otherBody.getAngle(),
              "radians (",
              ((otherBody.getAngle() * 180) / Math.PI).toFixed(1),
              "degrees)"
            );

            // Distance between centers
            const dx = currentPos.x - otherPosition.x;
            const dy = currentPos.y - otherPosition.y;
            const distance = Math.sqrt(dx * dx + dy * dy);
            console.log(
              "  Distance between centers:",
              distance.toFixed(3),
              "m"
            );
          } else {
            console.log(
              "  ⚠️ Velocity change detected but projectile NOT in this contact!"
            );
            console.log(
              "  This might be a different collision affecting the system."
            );
          }
        }
        console.log("---");
      }
    });

    // Sling (distance joint - soft enough to prevent wild initial forces)
    const slingJoint = this.simulator.world.createJoint(
      planck.DistanceJoint({
        bodyA: arm,
        bodyB: projectile,
        localAnchorA: planck.Vec2(-leftArmLength, 0),
        localAnchorB: planck.Vec2(0, 0),
        length: slingLength,
        dampingRatio: 0.5, // Lower damping to allow more give
        frequencyHz: 3.0, // Much softer spring
      })
    );
    joints.push(slingJoint);

    return {
      bodies,
      joints,
      projectile,
      slingJoint,
    };
  }
}
