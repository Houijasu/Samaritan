namespace Samaritan.Simulation.Core;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Results;

/// <summary>
/// Tracks the current state of the simulation.
/// </summary>
public class SimulationState
{
    /// <summary>
    /// Current simulation time in seconds.
    /// </summary>
    public double Time { get; set; }

    /// <summary>
    /// Current phase of the simulation.
    /// </summary>
    public SimulationPhase Phase { get; set; }

    /// <summary>
    /// Current target position.
    /// </summary>
    public Point2D TargetPosition { get; set; }

    /// <summary>
    /// Current target velocity.
    /// </summary>
    public Vector2D TargetVelocity { get; set; }

    /// <summary>
    /// Current projectile position (if active).
    /// </summary>
    public Point2D? ProjectilePosition { get; set; }

    /// <summary>
    /// The prediction result computed at simulation start.
    /// </summary>
    public PredictionResult? Prediction { get; set; }

    /// <summary>
    /// Position where skillshot was aimed (from prediction).
    /// </summary>
    public Point2D? CastPosition { get; set; }

    /// <summary>
    /// Position where we predicted target would be at interception.
    /// </summary>
    public Point2D? PredictedTargetPosition { get; set; }

    /// <summary>
    /// Position predicted by the exact analytical method (for comparison).
    /// </summary>
    public Point2D? ExactPredictedPosition { get; set; }

    /// <summary>
    /// Time predicted by the exact analytical method (for comparison).
    /// </summary>
    public double? ExactPredictedTime { get; set; }

    /// <summary>
    /// Cosine of angle between diff vector and target velocity (interception geometry).
    /// </summary>
    public double? CosTheta { get; set; }

    /// <summary>
    /// Sine of angle between diff vector and target velocity (interception geometry).
    /// </summary>
    public double? SinTheta { get; set; }

    /// <summary>
    /// Actual time when hit occurred (if any).
    /// </summary>
    public double? ActualHitTime { get; set; }

    /// <summary>
    /// Actual position where hit occurred (if any).
    /// </summary>
    public Point2D? ActualHitPosition { get; set; }

    /// <summary>
    /// Whether the projectile has been launched.
    /// </summary>
    public bool ProjectileLaunched { get; set; }

    /// <summary>
    /// Time when projectile was launched (after delay).
    /// </summary>
    public double ProjectileLaunchTime { get; set; }

    /// <summary>
    /// Position error: distance between predicted and actual hit positions.
    /// </summary>
    public double? PositionError =>
        ActualHitPosition.HasValue && PredictedTargetPosition.HasValue
            ? ActualHitPosition.Value.DistanceTo(PredictedTargetPosition.Value)
            : null;

    /// <summary>
    /// Time error: difference between predicted and actual hit times.
    /// </summary>
    public double? TimeError
    {
        get
        {
            if (!ActualHitTime.HasValue) return null;
            if (Prediction is null) return null;

            return Prediction.Match(
                hit: h => (double?)Math.Abs(ActualHitTime.Value - h.InterceptionTime),
                outOfRange: _ => (double?)null,
                unreachable: _ => (double?)null);
        }
    }

    public void Reset()
    {
        Time = 0;
        Phase = SimulationPhase.Ready;
        TargetPosition = default;
        TargetVelocity = default;
        ProjectilePosition = null;
        Prediction = null;
        CastPosition = null;
        PredictedTargetPosition = null;
        ExactPredictedPosition = null;
        ExactPredictedTime = null;
        CosTheta = null;
        SinTheta = null;
        ActualHitTime = null;
        ActualHitPosition = null;
        ProjectileLaunched = false;
        ProjectileLaunchTime = 0;
    }
}

/// <summary>
/// Phases of the simulation.
/// </summary>
public enum SimulationPhase
{
    /// <summary>Ready to start, waiting for user input.</summary>
    Ready,

    /// <summary>Computing prediction and preparing to cast.</summary>
    Predicting,

    /// <summary>In cast delay, waiting to launch projectile.</summary>
    Casting,

    /// <summary>Projectile is flying toward target.</summary>
    Flying,

    /// <summary>Simulation complete (hit or miss determined).</summary>
    Complete
}
