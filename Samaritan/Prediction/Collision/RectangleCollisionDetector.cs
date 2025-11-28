namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Collision detector for rectangular skillshots.
/// </summary>
public sealed class RectangleCollisionDetector : ICollisionDetector
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
        if (skillshot is not Skillshot.Rectangle rect)
        {
            return false;
        }

        if (timeElapsed < rect.Delay)
        {
            return false;
        }

        // Calculate rectangle center based on travel time
        var travelTime = timeElapsed - rect.Delay;
        var distance = rect.Speed * travelTime;
        var clampedDistance = Math.Min(distance, rect.Range);

        var rectCenter = origin + aimDirection.ScaleBy(clampedDistance / 2.0);

        // Expand rectangle by hitbox radius
        var halfWidth = rect.Width / 2.0 + targetHitboxRadius;
        var halfLength = rect.Length / 2.0 + targetHitboxRadius;

        // Transform to local coordinates (rectangle aligned with axes)
        var toTarget = targetPosition - rectCenter;

        // Perpendicular direction
        var perpendicular = new Vector2D(-aimDirection.Y, aimDirection.X);

        // Project onto rectangle axes
        var localX = toTarget.DotProduct(aimDirection);
        var localY = toTarget.DotProduct(perpendicular);

        // Check if point is inside expanded rectangle
        return Math.Abs(localX) <= halfLength && Math.Abs(localY) <= halfWidth;
    }
}
