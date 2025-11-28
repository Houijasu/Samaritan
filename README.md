# Samaritan

A skillshot prediction engine for League of Legends with real-time visualization.

## Features

- **Multi-path interception** - Predicts interception points for targets following complex waypoint paths (zigzag, curves)
- **Trailing edge calculation** - Accounts for target hitbox and aims at optimal edge for maximum hit probability
- **Network compensation** - Adjusts predictions for ping and cast delay
- **Real-time simulation** - Visual testing environment built with MonoGame

## Projects

| Project | Description |
|---------|-------------|
| `Samaritan` | Core prediction engine library |
| `Samaritan.Simulation` | MonoGame visualization for testing predictions |
| `Samaritan.Tests` | Unit tests |

## Requirements

- .NET 10.0+
- MonoGame 3.8.4

## Usage

Run the simulation:

```bash
dotnet run --project Samaritan.Simulation
```

## Algorithm

The prediction engine uses a quadratic interception formula with path cutting:

1. **Path cutting**: Offset target path by `delay * speed - hitbox` to account for cast delay and hitbox
2. **Quadratic solve**: Find intersection time using `at² + bt + c = 0` where:
   - `a = v² - p²` (velocity squared minus projectile speed squared)
   - `b = 2(diff·v - p²·tTotal)` (accounts for accumulated time on multi-segment paths)
   - `c = diff² - p²·tTotal²`
3. **Aim point**: Position on cut path at interception time

## License

Private - All rights reserved
