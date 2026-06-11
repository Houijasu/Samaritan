namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Collision detector for vector-cast rectangular skillshots (Viktor E, Rumble R).
/// The beam starts at the aim position (clamped to cast range) and its damaging
/// front sweeps along the aim direction for up to MaxLength. The check is
/// time-aligned: the target must be crossed by the front within one server tick
/// of the queried moment.
/// </summary>
public sealed class VectorRectangleCollisionDetector : ICollisionDetector
{
    // Temporal resolution of the hit check; the server processes hits at tick boundaries
    private const double TickDurationSeconds = 1.0 / 30.0;

    /// <inheritdoc />
    public bool WillHit(
        Skillshot skillshot,
        Point2D origin,
        Point2D aimPosition,
        Point2D targetPosition,
        double targetHitboxRadius,
        double timeElapsed)
    {
        if (skillshot is not Skillshot.VectorRectangle vectorRect)
        {
            return false;
        }

        if (timeElapsed < vectorRect.Delay)
        {
            return false;
        }

        var aimVector = aimPosition - origin;
        if (aimVector.Length < 1e-9)
        {
            return false;
        }

        var direction = aimVector.Normalize();

        // The beam starts where it was aimed, clamped to the cast range
        var startDistance = Math.Min(aimVector.Length, vectorRect.Range);
        var start = origin + direction.ScaleBy(startDistance);

        var travelTime = timeElapsed - vectorRect.Delay;

        // Beam finished sweeping more than a tick ago
        if (vectorRect.Speed * (travelTime - TickDurationSeconds) > vectorRect.MaxLength)
        {
            return false;
        }

        // Front positions now and one tick ago, measured from the beam start
        var frontNow = Math.Min(vectorRect.Speed * travelTime, (double)vectorRect.MaxLength);
        var frontPrevious = Math.Clamp(
            vectorRect.Speed * (travelTime - TickDurationSeconds), 0, vectorRect.MaxLength);

        var halfWidth = vectorRect.Width / 2.0 + targetHitboxRadius;

        // Transform to local coordinates aligned with the beam
        var toTarget = targetPosition - start;
        var perpendicular = new Vector2D(-direction.Y, direction.X);
        var alongLength = toTarget.DotProduct(direction);
        var alongWidth = toTarget.DotProduct(perpendicular);

        // The front must be crossing the target right now (within one tick)
        return Math.Abs(alongWidth) <= halfWidth &&
               alongLength >= frontPrevious - targetHitboxRadius &&
               alongLength <= frontNow + targetHitboxRadius;
    }
}
