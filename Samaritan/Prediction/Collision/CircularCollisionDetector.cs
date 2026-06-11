namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Collision detector for circular (area) skillshots.
/// The detonation lands at the aim position (clamped to range); after the
/// projectile arrives there, any target inside the area counts as hit.
/// </summary>
public sealed class CircularCollisionDetector : ICollisionDetector
{
    /// <inheritdoc />
    public bool WillHit(
        Skillshot skillshot,
        Point2D origin,
        Point2D aimPosition,
        Point2D targetPosition,
        double targetHitboxRadius,
        double timeElapsed)
    {
        if (skillshot is not Skillshot.Circular circular)
        {
            return false;
        }

        if (timeElapsed < circular.Delay)
        {
            return false;
        }

        // Impact position is where the skillshot was aimed, clamped to cast range
        var aimVector = aimPosition - origin;
        var clampedDistance = Math.Min(aimVector.Length, circular.Range);
        var impactPosition = clampedDistance > 1e-9
            ? origin + aimVector.Normalize().ScaleBy(clampedDistance)
            : origin;

        // Check travel time to the impact position
        var travelTime = circular.Speed > 0 ? clampedDistance / circular.Speed : 0;
        var totalTime = circular.Delay + travelTime;

        if (timeElapsed < totalTime)
        {
            return false;
        }

        // Circle-circle intersection
        var effectiveRadius = circular.Radius + targetHitboxRadius;
        var distance = impactPosition.DistanceTo(targetPosition);

        return distance <= effectiveRadius;
    }
}
