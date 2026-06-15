# ADR-002: Physics Engine Bakeoff — Python vs TypeScript

## Status
Decided

## Context
The original Planck.js-based physics had persistent bugs (projectile clipping through ground, incorrect release timing, objects getting stuck). We needed a correct physics implementation and evaluated two approaches side-by-side.

## Candidates

### 1. Python (SciPy + Matplotlib)
- **Approach**: Lagrangian mechanics with `scipy.integrate.solve_ivp` (adaptive RK45)
- **Pros**: NumPy/SciPy are battle-tested for scientific computing; easy to validate against VirtualTrebuchet.com
- **Cons**: Requires Python backend + WebSocket bridge for browser; ~200ms per simulation (too slow for real-time interaction)
- **Result**: Produced correct physics (40.3 m/s release speed, 84.1m range vs VT's 39.9/86.2m) but latency was unacceptable for interactive use

### 2. TypeScript (Custom RK4)
- **Approach**: Same Lagrangian mechanics, hand-rolled fixed-step RK4 integrator
- **Pros**: Runs natively in browser, <5ms per simulation, no server dependency
- **Cons**: More code to maintain; no adaptive step size (mitigated by small fixed dt=0.0005s)
- **Result**: Matches Python output exactly; validated against VirtualTrebuchet.com within 2.5%

## Decision
**TypeScript with custom RK4 integrator.**

The Python implementation served as a validation reference during development but is too slow for interactive browser use. The TypeScript version produces identical results and runs 40x faster.

## Key Learnings
1. The physics equations are straightforward Lagrangian mechanics — no physics library needed
2. The critical challenge was getting **initial conditions** right (Aq0 = π - acos(h/LAl)), not the integration itself
3. Fixed-step RK4 at dt=0.0005s is more than sufficient for trebuchet timescales (~5s total)
4. VirtualTrebuchet.com's source code was the authoritative reference for coordinate conventions

## References
- VirtualTrebuchet.com (validation target)
- ADR-001 for physics engine architecture decision
- Python reference code retained locally in `bakeoff/` (not checked into repo)
