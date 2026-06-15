# Architecture Decision Record: Physics Engine

**Status**: Accepted  
**Date**: 2026-06-14  
**Decision**: Replace Planck.js with analytical Lagrangian physics (TypeScript)

---

## Context

The trebuchet simulator uses Planck.js (Box2D port) for physics. Users experience:
- Projectile getting caught or passing through ground
- Unnatural sling behavior (modeled as spring, not rope)
- Fragile release timing (velocity-angle based)
- Fixing one bug introduces another

These are inherent to using an iterative constraint solver for a well-defined mechanical system.

## Options Considered

### Option A: Keep Planck.js, fix constraint assembly
- **Pros**: No rewrite, familiar library
- **Cons**: Iterative solver fundamentally accumulates errors in coupled pendulum systems. The bugs are architectural, not implementation errors.
- **Verdict**: Rejected

### Option B: Python (Pyodide) + scipy.integrate.solve_ivp
- **Pros**: Battle-tested ODE solver, adaptive step-sizing, numpy for linear algebra
- **Cons**: 10MB initial download, 5-10s cold start, complex JS↔Python interop for rendering
- **Bake-off result**: Physics identical to TypeScript, but unacceptably slow page load
- **Verdict**: Rejected

### Option C: TypeScript with analytical Lagrangian physics ✅
- **Pros**: 
  - 31KB bundle, instant load
  - Same equations as VirtualTrebuchet.com (proven accurate)
  - Zero runtime dependencies
  - Physics decoupled from rendering (testable, debuggable)
  - RK4 at dt=0.001s is more than sufficient accuracy
- **Cons**: Hand-rolled RK4 (but it's ~20 lines, well-understood)
- **Verdict**: Accepted

## Decision

Use **TypeScript with analytical Lagrangian mechanics** for the physics engine.

### Key Design Principles

1. **Equations, not constraints** — The trebuchet is described by 3 angles (Aq, Wq, Sq) with exact differential equations derived from Lagrangian mechanics. No iterative solving.

2. **3-stage simulation**:
   - Stage 1: Fired, projectile on ground (2 DOF) or lifted (3 DOF)
   - Stage 2: Sling releases projectile (velocity angle = release angle)
   - Stage 3: Free flight (projectile motion with optional drag)

3. **Physics/rendering separation** — Physics produces arrays of positions over time. Renderer animates from those arrays. No coupling.

4. **Validation against VirtualTrebuchet.com** — Same equations, same parameters should produce same results (within <5% for range).

## Consequences

- Planck.js is removed as a dependency
- All trebuchet type builders need rewriting to use the new physics interface
- The collision-filtering, damping, and joint hacks are eliminated entirely
- Rendering becomes simpler (draw from XY coordinates, not iterate physics bodies)

## References

- VirtualTrebuchet.com equations: https://virtualtrebuchet.com/documentation/explanation/equationsofmotion/
- Bake-off implementations: `bakeoff/typescript/` and `bakeoff/python/`
- Full physics plan: `docs/physics-rewrite-plan.md`
