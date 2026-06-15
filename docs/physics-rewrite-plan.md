# Trebuchet Simulator — Physics Engine Rewrite Plan

## The Core Problem

**Planck.js (Box2D) is the wrong tool for this job.**

Box2D is a general-purpose collision/constraint solver — it approximates physics iteratively. A trebuchet is a well-defined mechanical system with known analytical equations of motion. VirtualTrebuchet.com gets its accuracy by solving the **exact Lagrangian differential equations** with Runge-Kutta integration — no collision detection, no constraint solving, no approximation errors accumulating over time.

This is why "fixing one thing breaks another" — Box2D's iterative solver introduces errors that compound across coupled bodies (arm + counterweight + sling + projectile). The more joints and constraints you add, the worse it gets.

## The Solution: Analytical Physics Engine (like VirtualTrebuchet)

**Ditch Planck.js for the physics. Keep it only if you want collision detection for cosmetic ground bouncing.**

The trebuchet is modeled as a **3-stage simulation** with different equations for each stage:

### Stage 1: Pre-Fire (Static / Cocked)
- The trebuchet is loaded and stationary
- Projectile may be on the ground OR on top of the arm (depends on trebuchet type)
- Counterweight is raised, system has potential energy stored
- **Transition to Stage 2**: user triggers "fire"

### Stage 2: Fired but Not Released (Mechanism in Motion)
- Counterweight drops, arm swings, sling whips around
- Projectile is still attached to the sling
- 3 degrees of freedom: arm angle (Aq) + counterweight angle (Wq) + sling angle (Sq)
- If projectile starts on ground: constrained to horizontal until lift-off, then full 3-DOF
- **Transition to Stage 3**: when projectile velocity vector angle matches release angle (sling releases)

### Stage 3: Free Flight
- Projectile is released from sling, flies through the air
- Simple projectile motion with optional air drag
- 2 DOF: Px, Py
- **End condition**: projectile height < 0 (hits ground)

## Why Not Use an Existing Physics Engine?

**Short answer**: You can — but not for the core trebuchet mechanics.

| Engine | Good For | Bad For Trebuchets |
|--------|----------|-------------------|
| Planck.js / Box2D | Collisions, platformers, ragdolls | Accumulates errors in coupled pendulum systems |
| Matter.js | Simple 2D games | Same issues, less accurate than Box2D |
| Cannon.js / Ammo.js | 3D physics | Overkill, same constraint-solver issues |

**The fundamental problem**: General-purpose physics engines use *iterative constraint solvers*. They approximate. For a trebuchet (which is a precisely coupled multi-pendulum), small errors compound each frame and produce the exact bugs you're seeing — things getting stuck, passing through ground, releasing at wrong times.

**What VirtualTrebuchet.com does** (and what we should do): Solve the *exact* differential equations of motion with a numerical ODE solver (RK4). No approximation, no iteration, mathematically stable.

**However**, you CAN use an existing ODE solver library instead of writing RK4 from scratch:
- `odex` (npm) — adaptive Gragg-Bulirsch-Stoer, good for non-stiff systems
- `mathjs` — has `solveODE` with RK45 (Dormand-Prince) built in
- `ode-rk4` (npm) — simple, purpose-built RK4

**Recommendation**: Use `odex` or `mathjs` for the ODE solving, write the trebuchet equations ourselves. This gives us proven numerical methods + accurate trebuchet physics.

## Collisions

Collisions in this simulator are minimal and mostly cosmetic:
- **Projectile ↔ Ground**: determines landing distance (Stage 3 end condition)
- **Arm ↔ Ground**: prevents arm from swinging below horizontal (if applicable)
- **Counterweight ↔ Ground**: prevents CW from going below ground

These can be handled as simple **geometric constraints** (if Y < groundLevel, clamp it) rather than a full collision engine. No need for Box2D just for this.

## DOF (Degrees of Freedom) Explained

DOF = the number of independent variables needed to fully describe where everything is.

- A trebuchet with arm + hanging CW + sling = **3 DOF** (3 angles: Aq, Wq, Sq)
- Once you know those 3 angles, you can calculate the exact X,Y position of every part
- Compare to Planck.js where you have ~10 bodies each with 3 variables (x, y, angle) = 30 variables, most of which are redundant and must be "constrained" (source of bugs)



```
Aq = Arm angle (from vertical, positive = counterweight side down)
Wq = Counterweight angle (relative to arm extension)
Sq = Sling angle (relative to arm)

LAl = Arm length (pivot to sling attachment, "long arm")
LAs = Arm length (pivot to CW attachment, "short arm")
LAcg = Arm center of gravity distance from pivot
LW  = Counterweight pivot to CW center of gravity (hanging length)
LS  = Sling length
h   = Pivot height above ground

mA = Arm mass
mW = Counterweight mass
mP = Projectile mass
IA3 = Arm moment of inertia about pivot
IW3 = CW moment of inertia about its pivot
```

## Equations of Motion (from VirtualTrebuchet documentation)

### Stage 1 (2 equations, projectile on ground):

The system is 2 coupled nonlinear ODEs solved as a matrix equation:

```
[M11  M12] [Aw']   [r1]
[M21  M22] [Ww'] = [r2]

Solution:
Aw' = (r1*M22 - r2*M12) / (M11*M22 - M12*M21)
Ww' = -(r1*M21 - r2*M11) / (M11*M22 - M12*M21)
```

Where M11, M12, M21, M22, r1, r2 are complex expressions of the current angles and angular velocities (full expressions documented on VirtualTrebuchet.com/documentation/explanation/equationsofmotion/).

### Stage 2 (3 equations, projectile free in sling):

```
[M11  M12  M13] [Aw']   [r1]
[M21  M22   0 ] [Ww'] = [r2]
[M31   0   M33] [Sw']   [r3]

Solution:
Aw' = -(r1*M22*M33 - r2*M12*M33 - r3*M13*M22) / (M13*M22*M31 - M33*(M11*M22 - M12*M21))
Ww' = (r1*M21*M33 - r2*(M11*M33 - M13*M31) - r3*M13*M21) / (M13*M22*M31 - M33*(M11*M22 - M12*M21))
Sw' = (r1*M22*M31 - r2*M12*M31 - r3*(M11*M22 - M12*M21)) / (M13*M22*M31 - M33*(M11*M22 - M12*M21))
```

### Stage 3 (projectile flight with drag):

```
Pvx' = -(ρ*Cd*Aeff*(Pvx-WS)*sqrt(Pvy² + (WS-Pvx)²)) / (2*mP)
Pvy' = -Grav - (ρ*Cd*Aeff*Pvy*sqrt(Pvy² + (WS-Pvx)²)) / (2*mP)
```

## Numerical Integration: RK4

All stages use **4th-order Runge-Kutta** to integrate the ODEs. This is stable, accurate, and well-suited for this problem. No iterative constraint solver needed.

## Architecture

```
src/
  physics/
    trebuchet-physics.js    — The ODE system (stages 1, 2, 3)
    rk4-integrator.js       — Generic RK4 solver
    trebuchet-geometry.js   — Angles → XY coordinate conversion
  renderer/
    canvas-renderer.js      — Draws the trebuchet from XY positions (no physics lib needed)
  app.js                    — UI, parameters, controls
```

**Key insight**: Physics and rendering are completely decoupled. The physics produces angles over time. The renderer converts angles → XY positions → canvas drawing. No physics engine renders anything.

## Implementation Phases

### Phase 1: RK4 Integrator
- Generic `rk4Step(f, y, t, dt)` function
- Test with simple pendulum to verify correctness

### Phase 2: Stage 2 Physics (most important)
- Implement the 3-DOF equations of motion
- This is where the projectile is in the sling swinging freely
- Validate: energy should be conserved (KE + PE = constant)

### Phase 3: Stage 1 Physics
- 2-DOF equations with ground constraint on projectile
- Transition detection: compute ground reaction force, trigger when it goes negative

### Phase 4: Stage 3 + Release
- Projectile motion with drag
- Release condition: velocity angle matches user-specified release angle

### Phase 5: Renderer
- Convert angles → XY using the geometry equations
- Draw arm, counterweight, sling, projectile as simple shapes
- Animate by stepping through time and redrawing

### Phase 6: Validation
- Compare against VirtualTrebuchet.com with same parameters
- Known test case: 200kg CW, 10kg projectile, 6m short arm, 14m long arm → should get ~100-150m

## Model & Prompt Recommendation

### Which AI Model

**Claude Opus** (or Sonnet 4.5) — this is a math-heavy implementation where getting the equations exactly right matters. Opus has the strongest mathematical reasoning.

### Prompt Strategy

Give it ONE prompt with:
1. The complete equations (copy from this plan)
2. The architecture (files, interfaces)
3. A test case with expected output
4. Explicit "DO NOT use Planck.js/Box2D for physics"

### The Prompt

```
You are implementing an accurate 2D trebuchet physics simulator in JavaScript.

## Approach
DO NOT use any physics engine (Box2D, Planck.js, Matter.js, etc.).
Instead, implement the analytical equations of motion derived from Lagrangian
mechanics and solve them numerically with 4th-order Runge-Kutta (RK4).

## The Simulation Has 3 Stages

### Stage 1: Projectile on ground
The projectile slides along the ground. The system has 2 DOF: arm angle (Aq)
and counterweight angle (Wq). The sling angle (Sq) is constrained by the
ground contact.

Transition to Stage 2: when the vertical force on the projectile from the
sling exceeds gravity (ground reaction force goes negative).

### Stage 2: Projectile in sling (free)
The projectile swings freely at the end of the sling. The system has 3 DOF:
Aq, Wq, Sq.

Transition to Stage 3: when the angle of the projectile's velocity vector
(relative to horizontal) matches the user-specified release angle.

### Stage 3: Free flight
Simple projectile motion. Optionally include air resistance:
  Pvx' = -(ρ*Cd*Aeff*(Pvx-WS)*sqrt(Pvy² + (WS-Pvx)²)) / (2*mP)
  Pvy' = -Grav - (ρ*Cd*Aeff*Pvy*sqrt(Pvy² + (WS-Pvx)²)) / (2*mP)

End condition: projectile Y position goes below ground (Y < 0).

## Parameters (all in SI units)
- LAl: arm length, pivot to sling attachment (long side) [m]
- LAs: arm length, pivot to CW attachment (short side) [m]
- LAcg: arm center of gravity from pivot [m] (positive toward short side)
- LW: CW hanging length (CW pivot to CW center of mass) [m]
- LS: sling length [m]
- h: pivot height above ground [m]
- mA: arm mass [kg]
- mW: counterweight mass [kg]
- mP: projectile mass [kg]
- IA3: arm moment of inertia about pivot [kg·m²]
- IW3: CW moment of inertia about its pivot [kg·m²]
- releaseAngle: angle of velocity vector at release [degrees]
- Grav: 9.81 m/s²

## Angle Convention (CRITICAL)
- Aq: arm angle measured from VERTICAL. Aq=0 means arm is vertical with
  long side pointing down. Positive Aq = counterweight side dropping.
- Wq: counterweight angle relative to the arm's short-side extension.
  Wq=0 means CW hangs straight along arm extension.
- Sq: sling angle relative to arm's long-side extension.
  Sq=0 means sling hangs straight along arm extension.
- All positions measured relative to the PIVOT POINT.

## Coordinate Conversion (Angles → XY)
Weight CG: X = LAs*sin(Aq) + LW*sin(Aq+Wq), Y = -LAs*cos(Aq) - LW*cos(Aq+Wq)
Arm/Sling Point: X = -LAl*sin(Aq), Y = LAl*cos(Aq)
Projectile: X = -LAl*sin(Aq) - LS*sin(Aq+Sq), Y = LAl*cos(Aq) + LS*cos(Aq+Sq)
(Y is positive downward from pivot, so actual height = h - Y)

## Stage 1 Equations of Motion
[Include the full M11, M12, M21, M22, r1, r2 expressions from VirtualTrebuchet]
Sw constraint: Sw = -(LAl*sin(Aq) + LS*sin(Aq+Sq))*Aw / (LS*sin(Aq+Sq))

## Stage 2 Equations of Motion
[Include the full 3x3 matrix system from VirtualTrebuchet]

## Numerical Method
Use RK4 with dt = 0.001s (1ms steps). Display at 60fps by stepping
multiple times per frame.

## Deliverables
1. `src/physics/rk4-integrator.js` — generic RK4 stepper
2. `src/physics/trebuchet-physics.js` — ODE definitions for all 3 stages,
   stage transition logic, full simulation runner
3. `src/physics/trebuchet-geometry.js` — angles to XY conversion
4. `src/renderer/canvas-renderer.js` — draws trebuchet from XY positions
5. Updated `app.js` — connects UI to new physics

## Validation
With parameters: LAl=6m, LAs=2m, LW=2m, LS=6m, h=5m, mA=50kg, mW=200kg,
mP=10kg, releaseAngle=45°, the projectile should travel approximately
80-150m. Compare with virtualtrebuchet.com.

## What NOT to do
- Do NOT use Planck.js, Box2D, Matter.js or any constraint-based physics engine
- Do NOT approximate the sling as a spring or distance constraint
- Do NOT use collision detection for ground contact — use the analytical
  Stage 1 → Stage 2 transition
- Do NOT use iterative solvers — use direct matrix inversion (2x2 and 3x3)
- Do NOT introduce damping unless it represents real physical friction
```

## Summary

| Current (Planck.js) | New (Analytical) |
|---------------------|------------------|
| Iterative constraint solver | Exact ODE solution |
| Errors compound over time | Mathematically stable (RK4) |
| Collision filtering hacks | Clean stage transitions |
| Velocity-based release (fragile) | Angle-based release (geometric) |
| Bodies can tunnel through ground | Ground is a constraint equation |
| ~10 interacting bodies | 3 angles fully describe the system |

