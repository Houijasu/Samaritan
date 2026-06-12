namespace Samaritan.Simulation.Scenarios;

using MathNet.Spatial.Euclidean;

using Samaritan.Simulation.Core;

/// <summary>
/// Built-in scenarios for testing prediction accuracy.
/// All scenarios use Nidalee Q (Javelin Toss); they differ by target movement.
/// </summary>
public static class BuiltInScenarios
{
    /// <summary>
    /// Nidalee Q (Javelin Toss): narrow, fast, long-range linear skillshot.
    /// </summary>
    private static Skillshot.Linear NidaleeQ => new(
        Delay: 0.25f, Speed: 1300, Width: 40, Range: 1500);

    /// <summary>
    /// Gets all built-in test scenarios.
    /// </summary>
    /// <returns>Array of all predefined scenarios.</returns>
    public static Scenario[] GetAll() =>
    [
        VsStationary(),
        VsWalkingPerpendicular(),
        VsWalkingAway(),
        VsWalkingDiagonal(),
        VsCloseTarget(),
        VsFastTarget(),
        VsWalkingCrossing(),
        VsWaypointPath(),
        VsZigzagPath()
    ];

    /// <summary>
    /// Stationary target - baseline accuracy test.
    /// </summary>
    public static Scenario VsStationary() => new(
        Name: "Nidalee Q vs Stationary",
        Skillshot: NidaleeQ,
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Stationary(new Point2D(700, 0)),
        HitboxRadius: 65);

    /// <summary>
    /// Target moving perpendicular - tests leading.
    /// </summary>
    public static Scenario VsWalkingPerpendicular() => new(
        Name: "Nidalee Q vs Walking (Perpendicular)",
        Skillshot: NidaleeQ,
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(600, -200),
            Velocity: new Vector2D(0, 350), // Moving upward
            Duration: 3.0),
        HitboxRadius: 65);

    /// <summary>
    /// Target running away - hardest to catch up to.
    /// </summary>
    public static Scenario VsWalkingAway() => new(
        Name: "Nidalee Q vs Walking (Away)",
        Skillshot: NidaleeQ,
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(500, 0),
            Velocity: new Vector2D(350, 0), // Running away
            Duration: 3.0),
        HitboxRadius: 65);

    /// <summary>
    /// Target moving diagonally toward the caster's side.
    /// </summary>
    public static Scenario VsWalkingDiagonal() => new(
        Name: "Nidalee Q vs Walking (Diagonal)",
        Skillshot: NidaleeQ,
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(700, 100),
            Velocity: new Vector2D(-200, 150),
            Duration: 3.0),
        HitboxRadius: 65);

    /// <summary>
    /// Close target moving across - short reaction window.
    /// </summary>
    public static Scenario VsCloseTarget() => new(
        Name: "Nidalee Q vs Close Target",
        Skillshot: NidaleeQ,
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(400, 50),
            Velocity: new Vector2D(100, 200),
            Duration: 2.0),
        HitboxRadius: 65);

    /// <summary>
    /// Fast diagonal target - high speed relative to the javelin.
    /// </summary>
    public static Scenario VsFastTarget() => new(
        Name: "Nidalee Q vs Fast Target",
        Skillshot: NidaleeQ,
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(600, -300),
            Velocity: new Vector2D(150, 450), // Fast diagonal movement
            Duration: 3.0),
        HitboxRadius: 65);

    /// <summary>
    /// Target crossing toward the line of fire at long range.
    /// </summary>
    public static Scenario VsWalkingCrossing() => new(
        Name: "Nidalee Q vs Walking (Crossing)",
        Skillshot: NidaleeQ,
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(600, 200),
            Velocity: new Vector2D(-100, 150),
            Duration: 3.0),
        HitboxRadius: 65);

    /// <summary>
    /// Target following an L-shaped waypoint path.
    /// Tests prediction when the target changes direction mid-path.
    /// </summary>
    public static Scenario VsWaypointPath() => new(
        Name: "Nidalee Q vs Waypoint Path",
        Skillshot: NidaleeQ,
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Waypoints(
            Points:
            [
                new Point2D(400, -300),  // Start
                new Point2D(400, 100),   // Move up
                new Point2D(800, 100)    // Turn right
            ],
            Speed: 350),
        HitboxRadius: 65);

    /// <summary>
    /// Target in a zigzag evasion pattern.
    /// Challenging scenario with multiple direction changes.
    /// </summary>
    public static Scenario VsZigzagPath() => new(
        Name: "Nidalee Q vs Zigzag Path",
        Skillshot: NidaleeQ,
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Waypoints(
            Points:
            [
                new Point2D(500, 0),      // Start
                new Point2D(600, 150),    // Zigzag up-right
                new Point2D(700, -100),   // Zigzag down-right
                new Point2D(800, 100),    // Zigzag up-right
                new Point2D(900, -50)     // Zigzag down-right
            ],
            Speed: 400),
        HitboxRadius: 65);
}
