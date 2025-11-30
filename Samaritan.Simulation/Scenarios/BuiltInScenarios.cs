namespace Samaritan.Simulation.Scenarios;

using MathNet.Spatial.Euclidean;

using Samaritan.Simulation.Core;

/// <summary>
/// Built-in scenarios for testing prediction accuracy.
/// </summary>
public static class BuiltInScenarios
{
    /// <summary>
    /// Gets all built-in test scenarios.
    /// </summary>
    /// <returns>Array of all predefined scenarios.</returns>
    public static Scenario[] GetAll() =>
    [
        LinearVsStationary(),
        LinearVsWalkingPerpendicular(),
        LinearVsWalkingAway(),
        CircularVsWalking(),
        ConeVsClose(),
        LinearVsFastTarget(),
        ArcVsWalking(),
        LinearVsWaypointPath(),
        LinearVsZigzagPath()
    ];

    /// <summary>
    /// Ezreal Q vs stationary target - baseline accuracy test.
    /// </summary>
    public static Scenario LinearVsStationary() => new(
        Name: "Linear vs Stationary",
        Skillshot: new Skillshot.Linear(Delay: 0.25f, Speed: 2000, Width: 60, Range: 1150),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Stationary(new Point2D(700, 0)),
        HitboxRadius: 65);

    /// <summary>
    /// Ezreal Q vs target moving perpendicular - tests leading.
    /// </summary>
    public static Scenario LinearVsWalkingPerpendicular() => new(
        Name: "Linear vs Walking (Perpendicular)",
        Skillshot: new Skillshot.Linear(Delay: 0.25f, Speed: 2000, Width: 60, Range: 1150),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(600, -200),
            Velocity: new Vector2D(0, 350), // Moving upward
            Duration: 3.0),
        HitboxRadius: 65);

    /// <summary>
    /// Ezreal Q vs target running away - hardest to hit.
    /// </summary>
    public static Scenario LinearVsWalkingAway() => new(
        Name: "Linear vs Walking (Away)",
        Skillshot: new Skillshot.Linear(Delay: 0.25f, Speed: 2000, Width: 60, Range: 1150),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(500, 0),
            Velocity: new Vector2D(350, 0), // Running away
            Duration: 3.0),
        HitboxRadius: 65);

    /// <summary>
    /// Lux E vs moving target - area damage prediction.
    /// </summary>
    public static Scenario CircularVsWalking() => new(
        Name: "Circular vs Walking",
        Skillshot: new Skillshot.Circular(Delay: 0.25f, Speed: 1200, Radius: 350, Range: 1100),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(700, 100),
            Velocity: new Vector2D(-200, 150),
            Duration: 3.0),
        HitboxRadius: 65);

    /// <summary>
    /// Annie W vs close target - cone accuracy.
    /// </summary>
    public static Scenario ConeVsClose() => new(
        Name: "Cone vs Close Target",
        Skillshot: new Skillshot.Cone(Delay: 0.25f, Angle: 50, Range: 600),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(400, 50),
            Velocity: new Vector2D(100, 200),
            Duration: 2.0),
        HitboxRadius: 65);

    /// <summary>
    /// Morgana Q vs fast target - slow skillshot vs fast movement.
    /// </summary>
    public static Scenario LinearVsFastTarget() => new(
        Name: "Linear vs Fast Target",
        Skillshot: new Skillshot.Linear(Delay: 0.25f, Speed: 1200, Width: 70, Range: 1175),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(600, -300),
            Velocity: new Vector2D(150, 450), // Fast diagonal movement
            Duration: 3.0),
        HitboxRadius: 65);

    /// <summary>
    /// Diana Q arc vs moving target.
    /// </summary>
    public static Scenario ArcVsWalking() => new(
        Name: "Arc vs Walking",
        Skillshot: new Skillshot.Arc(
            Delay: 0.25f,
            Speed: 1900,
            Width: 185,
            OuterRadius: 900,
            Angle: 250,
            Clockwise: false),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(600, 200),
            Velocity: new Vector2D(-100, 150),
            Duration: 3.0),
        HitboxRadius: 65);

    /// <summary>
    /// Ezreal Q vs target following L-shaped waypoint path.
    /// Tests prediction when target changes direction mid-path.
    /// </summary>
    public static Scenario LinearVsWaypointPath() => new(
        Name: "Linear vs Waypoint Path",
        Skillshot: new Skillshot.Linear(Delay: 0.25f, Speed: 2000, Width: 60, Range: 1150),
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
    /// Morgana Q vs target in zigzag evasion pattern.
    /// Challenging scenario with multiple direction changes.
    /// </summary>
    public static Scenario LinearVsZigzagPath() => new(
        Name: "Linear vs Zigzag Path",
        Skillshot: new Skillshot.Linear(Delay: 0.25f, Speed: 1200, Width: 70, Range: 1175),
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
