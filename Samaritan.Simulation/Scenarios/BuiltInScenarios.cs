namespace Samaritan.Simulation.Scenarios;

using MathNet.Spatial.Euclidean;

using Samaritan.Simulation.Core;

/// <summary>
/// Built-in scenarios for testing prediction accuracy.
/// </summary>
public static class BuiltInScenarios
{
    // Nidalee Q (Javelin Toss) spell values
    private const float NidQ_Delay = 0.25f;
    private const float NidQ_Speed = 1300f;
    private const float NidQ_Width = 40f;
    private const float NidQ_Range = 1500f;

    /// <summary>
    /// Creates the standard Nidalee Q skillshot.
    /// </summary>
    private static Skillshot.Linear NidaleeQ() =>
        new(Delay: NidQ_Delay, Speed: NidQ_Speed, Width: NidQ_Width, Range: NidQ_Range);

    /// <summary>
    /// Gets all built-in test scenarios.
    /// </summary>
    /// <returns>Array of all predefined scenarios.</returns>
    public static Scenario[] GetAll() =>
    [
        LinearVsStationary(),
        LinearVsWalkingPerpendicular(),
        LinearVsWalkingAway(),
        LinearVsWalkingToward(),
        LinearVsDiagonal45(),
        LinearVsDiagonal135(),
        LinearVsFastTarget(),
        LinearVsFastApproaching(),
        LinearFromAngle(),
        LinearVsMaxRange(),
        CircularVsWalking(),
        ConeVsClose(),
        ArcVsWalking(),
        LinearVsWaypointPath(),
        LinearVsZigzagPath(),
        LinearVsZigzagLong(),
        LinearVsZigzagWide(),
        LinearVsZigzagFast(),
        LinearVsZigzagAway(),
        LinearVsZigzagMaxRange(),
        LinearVsCircularStrafe()
    ];

    /// <summary>
    /// Nidalee Q vs stationary target - baseline accuracy test.
    /// </summary>
    public static Scenario LinearVsStationary() => new(
        Name: "Nidalee Q vs Stationary",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Stationary(new Point2D(1200, 0)),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q vs target moving perpendicular - tests leading.
    /// </summary>
    public static Scenario LinearVsWalkingPerpendicular() => new(
        Name: "Nidalee Q vs Walking (Perpendicular)",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(1100, -300),
            Velocity: new Vector2D(0, 350),
            Duration: 4.0),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q vs target running away - hardest to hit.
    /// </summary>
    public static Scenario LinearVsWalkingAway() => new(
        Name: "Nidalee Q vs Walking (Away)",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(900, 0),
            Velocity: new Vector2D(350, 0),
            Duration: 4.0),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q vs target running toward caster - easy interception.
    /// </summary>
    public static Scenario LinearVsWalkingToward() => new(
        Name: "Nidalee Q vs Walking (Toward)",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(1400, 0),
            Velocity: new Vector2D(-350, 0),
            Duration: 4.0),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q vs target moving at 45° angle - diagonal interception.
    /// </summary>
    public static Scenario LinearVsDiagonal45() => new(
        Name: "Nidalee Q vs Diagonal (45°)",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(1000, -400),
            Velocity: new Vector2D(247, 247),
            Duration: 4.0),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q vs target moving at 135° angle - diagonal away.
    /// </summary>
    public static Scenario LinearVsDiagonal135() => new(
        Name: "Nidalee Q vs Diagonal (135°)",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(900, -200),
            Velocity: new Vector2D(247, -247),
            Duration: 4.0),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q vs fast target approaching at angle.
    /// </summary>
    public static Scenario LinearVsFastApproaching() => new(
        Name: "Nidalee Q vs Fast (Approaching)",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(1300, 500),
            Velocity: new Vector2D(-400, -200),
            Duration: 4.0),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q from off-center position - tests non-origin caster.
    /// </summary>
    public static Scenario LinearFromAngle() => new(
        Name: "Nidalee Q from Angle",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(-400, -300),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(800, 300),
            Velocity: new Vector2D(200, -300),
            Duration: 4.0),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q at maximum range (1500 units) - tests edge of range.
    /// </summary>
    public static Scenario LinearVsMaxRange() => new(
        Name: "Nidalee Q vs Max Range",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(1400, -200),
            Velocity: new Vector2D(50, 250),
            Duration: 4.0),
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
    /// Nidalee Q vs fast target - slow skillshot vs fast movement.
    /// </summary>
    public static Scenario LinearVsFastTarget() => new(
        Name: "Nidalee Q vs Fast Target",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Linear(
            Start: new Point2D(1000, -400),
            Velocity: new Vector2D(150, 450),
            Duration: 4.0),
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
    /// Nidalee Q vs target following L-shaped waypoint path.
    /// Tests prediction when target changes direction mid-path.
    /// </summary>
    public static Scenario LinearVsWaypointPath() => new(
        Name: "Nidalee Q vs Waypoint Path",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Waypoints(
            Points:
            [
                new Point2D(900, -400),
                new Point2D(900, 200),
                new Point2D(1400, 200)
            ],
            Speed: 350),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q vs target in zigzag evasion pattern.
    /// Challenging scenario with multiple direction changes.
    /// </summary>
    public static Scenario LinearVsZigzagPath() => new(
        Name: "Nidalee Q vs Zigzag",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Waypoints(
            Points:
            [
                new Point2D(900, 0),
                new Point2D(1000, 200),
                new Point2D(1100, -180),
                new Point2D(1200, 180),
                new Point2D(1350, -120)
            ],
            Speed: 400),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q long distance zigzag - target zigzags toward max range.
    /// Tests prediction at extended ranges with direction changes.
    /// </summary>
    public static Scenario LinearVsZigzagLong() => new(
        Name: "Nidalee Q vs Zigzag (Long)",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Waypoints(
            Points:
            [
                new Point2D(800, 0),
                new Point2D(900, 250),
                new Point2D(1000, -200),
                new Point2D(1150, 230),
                new Point2D(1300, -180),
                new Point2D(1450, 150)
            ],
            Speed: 380),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q wide zigzag pattern - larger amplitude zigzags.
    /// Tests prediction with extreme direction changes.
    /// </summary>
    public static Scenario LinearVsZigzagWide() => new(
        Name: "Nidalee Q vs Zigzag (Wide)",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Waypoints(
            Points:
            [
                new Point2D(850, -400),
                new Point2D(1000, 400),
                new Point2D(1150, -380),
                new Point2D(1300, 350),
                new Point2D(1450, -300)
            ],
            Speed: 420),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q fast zigzag - high speed target with rapid direction changes.
    /// Very challenging interception scenario.
    /// </summary>
    public static Scenario LinearVsZigzagFast() => new(
        Name: "Nidalee Q vs Zigzag (Fast)",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Waypoints(
            Points:
            [
                new Point2D(850, 0),
                new Point2D(950, 160),
                new Point2D(1050, -140),
                new Point2D(1150, 170),
                new Point2D(1250, -120),
                new Point2D(1350, 150),
                new Point2D(1450, -100)
            ],
            Speed: 500),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q zigzag moving away from caster - evasive retreat pattern.
    /// Target maintains distance while zigzagging.
    /// </summary>
    public static Scenario LinearVsZigzagAway() => new(
        Name: "Nidalee Q vs Zigzag (Away)",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Waypoints(
            Points:
            [
                new Point2D(800, 0),
                new Point2D(920, 200),
                new Point2D(1040, -180),
                new Point2D(1160, 190),
                new Point2D(1280, -150),
                new Point2D(1400, 120)
            ],
            Speed: 400),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q zigzag at maximum range (1500 units).
    /// Tests prediction at edge of range with direction changes.
    /// </summary>
    public static Scenario LinearVsZigzagMaxRange() => new(
        Name: "Nidalee Q vs Zigzag (Max Range)",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Waypoints(
            Points:
            [
                new Point2D(1000, 0),
                new Point2D(1100, 220),
                new Point2D(1200, -200),
                new Point2D(1300, 240),
                new Point2D(1400, -180),
                new Point2D(1500, 120)
            ],
            Speed: 380),
        HitboxRadius: 65);

    /// <summary>
    /// Nidalee Q vs target strafing in a circular arc around caster.
    /// Tests prediction when target maintains constant distance.
    /// </summary>
    public static Scenario LinearVsCircularStrafe() => new(
        Name: "Nidalee Q vs Circular Strafe",
        Skillshot: NidaleeQ(),
        CasterPosition: new Point2D(0, 0),
        TargetMovement: new MovementPattern.Waypoints(
            Points:
            [
                new Point2D(1000, -500),
                new Point2D(1100, -200),
                new Point2D(1100, 200),
                new Point2D(1000, 500),
                new Point2D(800, 600)
            ],
            Speed: 380),
        HitboxRadius: 65);
}
