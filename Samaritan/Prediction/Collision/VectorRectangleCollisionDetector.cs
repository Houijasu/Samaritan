namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Collision detector for vector-cast rectangular skillshots (Viktor E, Rumble R).
/// These skillshots extend from caster toward a target direction with configurable length.
/// </summary>
public sealed class VectorRectangleCollisionDetector : ICollisionDetector
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
        if (skillshot is not Skillshot.VectorRectangle vectorRect)
        {
            return false;
        }

        if (timeElapsed < vectorRect.Delay)
        {
            return false;
        }

        // Calculate how far the projectile has traveled along the vector
        var travelTime = timeElapsed - vectorRect.Delay;
        var travelDistance = vectorRect.Speed * travelTime;

        // Clamp to max length
        var currentLength = Math.Min(travelDistance, vectorRect.MaxLength);

        // The rectangle extends from origin in the aim direction
        // Start position is at origin, end position is at origin + aimDirection * currentLength
        var endPosition = origin + aimDirection.ScaleBy(currentLength);

        // Expand half-width by hitbox radius
        var halfWidth = vectorRect.Width / 2.0 + targetHitboxRadius;

        // Check if target is within the rectangle
        // Transform to local coordinates aligned with rectangle
        var toTarget = targetPosition - origin;

        // Perpendicular direction (for width check)
        var perpendicular = new Vector2D(-aimDirection.Y, aimDirection.X);

        // Project onto rectangle axes
        var alongLength = toTarget.DotProduct(aimDirection);
        var alongWidth = toTarget.DotProduct(perpendicular);

        // Target must be:
        // 1. Between 0 and currentLength along the direction
        // 2. Within halfWidth perpendicular to the direction
        return alongLength >= -targetHitboxRadius &&
               alongLength <= currentLength + targetHitboxRadius &&
               Math.Abs(alongWidth) <= halfWidth;
    }
}
