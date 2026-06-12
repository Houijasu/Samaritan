# Samaritan

A skillshot prediction engine for League of Legends with real-time visualization.

## Features

- **Exact interception math** - Closed-form quadratic solves for moving targets, with no
  heuristic correction factors or numerical poles
- **Selectable aim modes** - Rear-edge graze (dodge-resistant), tangent graze (minimal
  penetration), or optimal (earliest rear-side contact with a penetration cap)
- **Multi-path interception** - Predicts interception points for targets following complex
  waypoint paths (zigzag, curves) via segment-by-segment path cutting
- **Continuous collision** - Swept relative-motion hit detection; grazing contacts cannot
  slip between simulation frames
- **Network compensation** - Adjusts predictions for ping, server tick, and cast delay
- **Real-time simulation** - Visual A/B testing environment built with MonoGame, with a
  live graze-margin readout and selectable prediction methods for comparison

## Projects

| Project | Description |
|---------|-------------|
| `Samaritan` | Core prediction engine library |
| `Samaritan.Simulation` | MonoGame visualization for testing predictions |
| `Samaritan.Tests` | Unit tests (xUnit) |
| `Samaritan.Benchmarks` | BenchmarkDotNet comparison of the prediction techniques |

## Requirements

- .NET 10.0+
- MonoGame 3.8.4 (the MGCB content tool is restored automatically via `.config/dotnet-tools.json`)

## Usage

Run the simulation:

```bash
dotnet run --project Samaritan.Simulation
```

### Simulation controls

| Input | Action |
|---|---|
| Space | Play / pause (restarts after a completed run) |
| R | Reset current scenario |
| Left / Right | Previous / next scenario |
| + / - | Simulation speed (0.25x - 4x) |
| **M** | Cycle prediction method: AFTER (exact rear) / NEAREST (tangent) / OPTIMAL (fast rear) / GAGONG (lua port) |
| Esc | Exit |
| Mouse drag | Yellow handles move the caster / target start; the green handle sets the target's movement direction |

The HUD shows the prediction, the exact-method comparison, the actual hit, and a
**Graze** line: the missile's closest approach to the target center over the simulated
flight versus the effective radius, i.e. how far the shot is from the tangency boundary.

## Aim modes (projectile skillshots vs moving targets)

| Mode | Goal | Behavior |
|---|---|---|
| `RearGraze` (default) | Exact-time rear contact | First contact at the exact (minimal) interception time, with the touch point swung toward the rear rim - rear-ness at zero time cost, using the flat region around the time minimum. Falls back to the rear-edge tangency graze. |
| `NearestRear` | Most tangent hit | Searches the missile ray angle directly for a closest approach of `R*(1 - eps)` in the actual-cast frame - the simulated "HIT by x" margin is ~0.3 units at any geometry where a tangent graze is reachable. |
| `Optimal` | Smallest interception time, still from behind | Among rays whose pass lands on the rear half of the hitbox and penetrates no deeper than the target's bounding radius, picks the earliest first contact and casts at the contact point itself. |

All modes fall back gracefully (`OutOfRange` / `Unreachable` / rear-graze) when their
constraint set is empty. Placed effects (circular, rectangle, vector) always center the
detonation on the predicted target position instead.

The simulation additionally offers **GAGONG**, a faithful port of a community Lua
prediction routine, kept as a comparison reference; its hot path is scalarized and
allocation-free (behavior pinned by golden-value tests). Benchmarks (Nidalee Q, per
call, caching disabled): Gagong ~160 ns, NearestRear ~260 ns, RearGraze ~420 ns,
Optimal ~2.8 us.

## Algorithm

**Straight-line targets** are solved in relative space. With caster `C`, target position
`P`, velocity `V` (speed `v`), projectile speed `p`, cast delay `d`, and effective radius
`R` (width/2 + target hitbox), the center-minus-front offset along a ray `r̂` is affine in
flight time: `g(s) = G0 + W*s` with `G0 = (P - C) + V*d` and `W = V - p*r̂`, so closest
approach, first contact, and tangency are all closed forms:

- Rear-graze lead: `L = R*sqrt(p^2 + v^2 - 2pv*cos(phi)) / (p*sin(phi))` (phi = angle
  between path and ray), with the reported time being the first contact - predicted and
  actual hit times agree.
- Tangent / optimal modes bisect or sweep the ray angle against the range-clamped
  closest-approach function - the same quantity the simulator's Graze readout measures.

**Waypoint paths** (zigzags, turns) are solved segment by segment with path cutting:
offset the path by `delay * speed - effectiveRadius`, then solve the per-segment quadratic
`a = v^2 - p^2`, `b = 2(diff.v - p^2*tTotal)`, `c = diff^2 - p^2*tTotal^2` and aim at the
cut-path position at the interception time.

**Instant skillshots** (cones, zero-speed AoE) apply at the end of the delay and aim at
the position the target reaches by then.

**Network compensation** (ping + half a server tick + optional reaction buffer) is added
to every cast delay. **Hit validation** is time-aligned: linear missiles are checked
against a continuous swept segment of both the missile's and the target's motion, so a
target in the wake behind the projectile never registers and thin grazes never slip
between frames.

## License

Private - All rights reserved
