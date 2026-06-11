namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Service that validates skillshot hits using the appropriate collision detector.
/// </summary>
public sealed class CollisionValidationService
{
    private readonly Dictionary<Type, ICollisionDetector> _detectors;

    /// <summary>
    /// Creates a collision validation service with all built-in detectors.
    /// </summary>
    public CollisionValidationService()
    {
        _detectors = new Dictionary<Type, ICollisionDetector>
        {
            [typeof(Skillshot.Linear)] = new LinearCollisionDetector(),
            [typeof(Skillshot.Circular)] = new CircularCollisionDetector(),
            [typeof(Skillshot.Cone)] = new ConeCollisionDetector(),
            [typeof(Skillshot.Arc)] = new ArcCollisionDetector(),
            [typeof(Skillshot.Rectangle)] = new RectangleCollisionDetector(),
            [typeof(Skillshot.VectorRectangle)] = new VectorRectangleCollisionDetector()
        };
    }

    /// <summary>
    /// Gets the appropriate collision detector for a skillshot type.
    /// </summary>
    public ICollisionDetector? GetDetector(Skillshot skillshot)
    {
        return _detectors.GetValueOrDefault(skillshot.GetType());
    }

    /// <summary>
    /// Validates whether a skillshot aimed at a position is hitting a target.
    /// </summary>
    /// <param name="skillshot">The skillshot to validate.</param>
    /// <param name="casterPosition">Position where the skillshot is cast from.</param>
    /// <param name="aimPosition">Position the skillshot is aimed at.</param>
    /// <param name="targetPosition">Current position of the target.</param>
    /// <param name="targetHitboxRadius">Target's hitbox radius.</param>
    /// <param name="timeElapsed">Time since skillshot was cast.</param>
    /// <returns>True if the skillshot will hit.</returns>
    public bool ValidateHit(
        Skillshot skillshot,
        Point2D casterPosition,
        Point2D aimPosition,
        Point2D targetPosition,
        double targetHitboxRadius,
        double timeElapsed)
    {
        var detector = GetDetector(skillshot);
        if (detector is null)
        {
            return false;
        }

        return detector.WillHit(skillshot, casterPosition, aimPosition, targetPosition, targetHitboxRadius, timeElapsed);
    }

    /// <summary>
    /// Validates whether a skillshot aimed at a position will hit a target at the predicted time.
    /// Uses the predicted interception time for validation.
    /// </summary>
    /// <param name="skillshot">The skillshot to validate.</param>
    /// <param name="casterPosition">Position where the skillshot is cast from.</param>
    /// <param name="predictedPosition">Predicted position where target will be intercepted.</param>
    /// <param name="targetHitboxRadius">Target's hitbox radius.</param>
    /// <param name="interceptionTime">Time when interception occurs.</param>
    /// <returns>True if the skillshot will hit at the predicted position and time.</returns>
    public bool ValidatePrediction(
        Skillshot skillshot,
        Point2D casterPosition,
        Point2D predictedPosition,
        double targetHitboxRadius,
        double interceptionTime)
    {
        var detector = GetDetector(skillshot);
        if (detector is null)
        {
            return false;
        }

        return detector.WillHit(skillshot, casterPosition, predictedPosition, predictedPosition, targetHitboxRadius, interceptionTime);
    }

    /// <summary>
    /// Simulates collision over time to find the first hit time.
    /// </summary>
    /// <param name="skillshot">The skillshot to check.</param>
    /// <param name="casterPosition">Position where the skillshot is cast from.</param>
    /// <param name="aimPosition">Position the skillshot is aimed at.</param>
    /// <param name="targetPosition">Target position at time 0.</param>
    /// <param name="targetVelocity">Target velocity.</param>
    /// <param name="targetHitboxRadius">Target's hitbox radius.</param>
    /// <param name="maxTime">Maximum time to simulate.</param>
    /// <param name="timeStep">Time step for simulation.</param>
    /// <returns>First hit time, or null if no hit occurs.</returns>
    public double? SimulateCollision(
        Skillshot skillshot,
        Point2D casterPosition,
        Point2D aimPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double targetHitboxRadius,
        double maxTime = 5.0,
        double timeStep = 0.016)
    {
        var detector = GetDetector(skillshot);
        if (detector is null)
        {
            return null;
        }

        for (var t = 0.0; t <= maxTime; t += timeStep)
        {
            var currentTargetPos = targetPosition + targetVelocity.ScaleBy(t);
            if (detector.WillHit(skillshot, casterPosition, aimPosition, currentTargetPos, targetHitboxRadius, t))
            {
                return t;
            }
        }

        return null;
    }
}
