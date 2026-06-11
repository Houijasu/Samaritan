namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Collision detector for arc-shaped skillshots.
/// </summary>
public sealed class ArcCollisionDetector : ICollisionDetector
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
        if (skillshot is not Skillshot.Arc arc)
        {
            return false;
        }

        if (timeElapsed < arc.Delay)
        {
            return false;
        }

        var aimVector = aimPosition - origin;
        if (aimVector.Length < 1e-9)
        {
            return false;
        }

        var aimDirection = aimVector.Normalize();
        var effectiveWidth = arc.Width / 2.0 + targetHitboxRadius;

        // Calculate how far along the arc the skillshot has traveled
        var travelTime = timeElapsed - arc.Delay;
        var angularSpeed = arc.Speed / arc.OuterRadius; // radians per second
        var currentAngle = angularSpeed * travelTime;

        // Clamp to maximum arc angle
        var maxAngleRadians = arc.Angle * Math.PI / 180.0;
        currentAngle = Math.Min(currentAngle, maxAngleRadians);

        // Sample points along the arc to check collision
        var samples = Math.Max(8, (int)(currentAngle * 10));
        var angleStep = currentAngle / samples;

        // Get initial angle from aim direction
        var startAngle = Math.Atan2(aimDirection.Y, aimDirection.X);

        // Direction multiplier: +1 for counter-clockwise, -1 for clockwise
        var directionMultiplier = arc.Clockwise ? -1.0 : 1.0;

        for (var i = 0; i <= samples; i++)
        {
            var angle = startAngle + directionMultiplier * angleStep * i;
            var arcPoint = new Point2D(
                origin.X + Math.Cos(angle) * arc.OuterRadius,
                origin.Y + Math.Sin(angle) * arc.OuterRadius);

            var distance = arcPoint.DistanceTo(targetPosition);
            if (distance <= effectiveWidth)
            {
                return true;
            }
        }

        // Also check inner edge of arc
        var innerRadius = arc.OuterRadius - arc.Width;
        if (innerRadius > 0)
        {
            for (var i = 0; i <= samples; i++)
            {
                var angle = startAngle + directionMultiplier * angleStep * i;
                var arcPoint = new Point2D(
                    origin.X + Math.Cos(angle) * innerRadius,
                    origin.Y + Math.Sin(angle) * innerRadius);

                var distance = arcPoint.DistanceTo(targetPosition);
                if (distance <= effectiveWidth)
                {
                    return true;
                }
            }
        }

        return false;
    }
}
