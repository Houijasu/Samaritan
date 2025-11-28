namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Collision detector for linear (line) skillshots.
/// </summary>
public sealed class LinearCollisionDetector : ICollisionDetector
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
        if (skillshot is not Skillshot.Linear linear)
        {
            return false;
        }

        if (timeElapsed < linear.Delay)
        {
            return false;
        }

        var effectiveRadius = linear.Width / 2.0 + targetHitboxRadius;

        // Calculate skillshot endpoint based on travel time
        var travelTime = timeElapsed - linear.Delay;
        var skillshotDistance = linear.Speed * travelTime;
        var clampedDistance = Math.Min(skillshotDistance, linear.Range);

        var skillshotEnd = origin + aimDirection.ScaleBy(clampedDistance);

        // Point-to-line-segment distance
        var distance = PointToSegmentDistance(targetPosition, origin, skillshotEnd);

        return distance <= effectiveRadius;
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
