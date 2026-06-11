namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Collision detector for cone-shaped skillshots.
/// </summary>
public sealed class ConeCollisionDetector : ICollisionDetector
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
        if (skillshot is not Skillshot.Cone cone)
        {
            return false;
        }

        if (timeElapsed < cone.Delay)
        {
            return false;
        }

        var aimVector = aimPosition - origin;
        if (aimVector.Length < 1e-9)
        {
            return false;
        }

        var aimDirection = aimVector.Normalize();

        var toTarget = targetPosition - origin;
        var distance = toTarget.Length;

        // Check range (add hitbox radius for edge cases)
        if (distance > cone.Range + targetHitboxRadius)
        {
            return false;
        }

        // Check angle
        if (distance < 0.001)
        {
            // Target is at the origin point, always considered a hit
            return true;
        }

        var targetDir = toTarget.Normalize();
        var dot = aimDirection.DotProduct(targetDir);
        var angleDegrees = Math.Acos(Math.Clamp(dot, -1.0, 1.0)) * 180.0 / Math.PI;

        // Account for hitbox by calculating angle offset
        var hitboxAngleOffset = distance > targetHitboxRadius
            ? Math.Asin(targetHitboxRadius / distance) * 180.0 / Math.PI
            : 90.0;

        var halfConeAngle = cone.Angle / 2.0;

        return angleDegrees <= halfConeAngle + hitboxAngleOffset;
    }
}
