namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Collision detector for linear (line) skillshots.
/// The check is time-aligned: the target must be touching the projectile head
/// (within one server tick of sweep) at the queried moment - a target standing
/// in the wake the projectile passed earlier does not register as a hit.
/// </summary>
public sealed class LinearCollisionDetector : ICollisionDetector
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
        if (skillshot is not Skillshot.Linear linear)
        {
            return false;
        }

        if (timeElapsed < linear.Delay)
        {
            return false;
        }

        var aimVector = aimPosition - origin;
        if (aimVector.Length < 1e-9)
        {
            return false;
        }

        var direction = aimVector.Normalize();
        var effectiveRadius = linear.Width / 2.0 + targetHitboxRadius;
        var travelTime = timeElapsed - linear.Delay;

        // Missile expired: the head passed max range more than a tick ago
        if (linear.Speed * (travelTime - TickDurationSeconds) > linear.Range)
        {
            return false;
        }

        // Segment swept by the head during the last tick
        var headDistance = Math.Min(linear.Speed * travelTime, (double)linear.Range);
        var tailDistance = Math.Clamp(linear.Speed * (travelTime - TickDurationSeconds), 0, linear.Range);

        var head = origin + direction.ScaleBy(headDistance);
        var tail = origin + direction.ScaleBy(tailDistance);

        return PointToSegmentDistance(targetPosition, tail, head) <= effectiveRadius;
    }

    /// <summary>
    /// Continuous collision between two moving points over one simulation step.
    /// <paramref name="relativeStart"/> and <paramref name="relativeEnd"/> are the
    /// offsets (front minus target center) at the start and end of the step; the
    /// relative motion is assumed linear in between. Returns true when the offset
    /// segment passes within <paramref name="radius"/> of the origin, with
    /// <paramref name="fraction"/> set to the earliest moment of contact (0..1).
    /// Catches grazing contacts that fall entirely between two discrete checks.
    /// </summary>
    public static bool SweptContact(
        Vector2D relativeStart,
        Vector2D relativeEnd,
        double radius,
        out double fraction)
    {
        fraction = 0;

        // Already touching at the start of the step
        var c = relativeStart.DotProduct(relativeStart) - radius * radius;
        if (c <= 0)
        {
            return true;
        }

        var delta = relativeEnd - relativeStart;
        var a = delta.DotProduct(delta);
        if (a < 1e-12)
        {
            return false;
        }

        // |relativeStart + delta*s| = radius  =>  a*s² + b*s + c = 0
        var b = 2.0 * relativeStart.DotProduct(delta);
        var discriminant = b * b - 4 * a * c;
        if (discriminant < 0)
        {
            return false;
        }

        var entry = (-b - Math.Sqrt(discriminant)) / (2 * a);
        if (entry < 0 || entry > 1)
        {
            return false;
        }

        fraction = entry;
        return true;
    }

    /// <summary>
    /// Calculates the distance from a point to a line segment.
    /// </summary>
    public static double PointToSegmentDistance(Point2D point, Point2D lineStart, Point2D lineEnd)
    {
        var line = lineEnd - lineStart;
        var lengthSq = line.DotProduct(line);

        if (lengthSq < 0.0001)
        {
            return point.DistanceTo(lineStart);
        }

        // Project point onto line, clamping to segment
        var pointToStart = point - lineStart;
        var t = Math.Clamp(pointToStart.DotProduct(line) / lengthSq, 0.0, 1.0);
        var projection = lineStart + line.ScaleBy(t);

        return point.DistanceTo(projection);
    }
}
