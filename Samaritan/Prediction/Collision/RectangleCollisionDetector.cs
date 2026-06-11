namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Collision detector for rectangular skillshots (placed area effects like
/// Anivia W / Karthus W). The rectangle is centered at the aim position
/// (clamped to range), oriented along the aim direction, and activates once
/// the cast delay and travel time have elapsed.
/// </summary>
public sealed class RectangleCollisionDetector : ICollisionDetector
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
        if (skillshot is not Skillshot.Rectangle rect)
        {
            return false;
        }

        if (timeElapsed < rect.Delay)
        {
            return false;
        }

        var aimVector = aimPosition - origin;
        if (aimVector.Length < 1e-9)
        {
            return false;
        }

        var direction = aimVector.Normalize();
        var clampedDistance = Math.Min(aimVector.Length, rect.Range);
        var rectCenter = origin + direction.ScaleBy(clampedDistance);

        // The effect activates after the cast delay plus travel to the landing point
        var travelTime = rect.Speed > 0 ? clampedDistance / rect.Speed : 0;
        if (timeElapsed < rect.Delay + travelTime)
        {
            return false;
        }

        // Expand rectangle by hitbox radius
        var halfWidth = rect.Width / 2.0 + targetHitboxRadius;
        var halfLength = rect.Length / 2.0 + targetHitboxRadius;

        // Transform to local coordinates (rectangle aligned with axes)
        var toTarget = targetPosition - rectCenter;

        // Perpendicular direction
        var perpendicular = new Vector2D(-direction.Y, direction.X);

        // Project onto rectangle axes
        var localX = toTarget.DotProduct(direction);
        var localY = toTarget.DotProduct(perpendicular);

        // Check if point is inside expanded rectangle
        return Math.Abs(localX) <= halfLength && Math.Abs(localY) <= halfWidth;
    }
}
