namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Collision detector for circular (area) skillshots.
/// </summary>
public sealed class CircularCollisionDetector : ICollisionDetector
{
    /// <inheritdoc />
    public bool WillHit(
        Skillshot skillshot,
        Point2D origin,
        Vector2D aimDirection,
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

        // Calculate impact position (where skillshot lands)
        var aimDistance = origin.DistanceTo(targetPosition);
        var clampedDistance = Math.Min(aimDistance, circular.Range);
        var impactPosition = origin + aimDirection.ScaleBy(clampedDistance);

        // Check travel time
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
