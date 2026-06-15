# Bake-Off: TypeScript vs Python (Pyodide)

## Goal
Build the same trebuchet simulator in both TypeScript and Python, 
then compare against VirtualTrebuchet.com with identical parameters.

## Test Parameters (from VirtualTrebuchet.com defaults)
- Arm length (long side): 6 m
- Arm length (short side): 2 m  
- Arm mass: 50 kg
- Counterweight mass: 200 kg
- CW hanging length: 2 m
- Sling length: 6 m
- Projectile mass: 10 kg
- Pivot height: 5 m
- Release angle: 45°

## Scoring Criteria
| Category | Weight | How We Measure |
|----------|--------|---------------|
| Accuracy | 40% | % error vs VirtualTrebuchet.com distance |
| Code Simplicity | 20% | Lines of code, readability, extensibility |
| Performance | 20% | Page load time, simulation compute time |
| UX | 20% | Responsiveness, animation smoothness |

## What Each Implementation Must Include
- Full physics (3-stage simulation)
- Canvas rendering (animated trebuchet firing)
- Parameter inputs (arm lengths, masses, sling, release angle)
- Stats display (distance, max height, velocity)
- Fire / Reset controls

## How to Run
- TypeScript: `cd bakeoff/typescript && npm install && npm start`
- Python: `cd bakeoff/python && python -m http.server 8081` (uses Pyodide CDN)
