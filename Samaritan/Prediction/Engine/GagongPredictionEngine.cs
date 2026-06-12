namespace Samaritan.Prediction.Engine;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Configuration;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Faithful C# adaptation of the "Gagong" Lua prediction routine (community
/// League scripting library), preserved as a comparison method:
/// <list type="bullet">
/// <item><c>mathf.project</c>: per-segment interception quadratic
/// |D + v·t| = v1·t with no delay or hitbox terms.</item>
/// <item><c>project</c>: waypoint walker - the cast delay is handled by
/// advancing/extending each segment start (virtual starts) instead of carrying
/// accumulated-time terms in the quadratic, with [minT, maxT] validity windows
/// per segment.</item>
/// <item><c>mod</c>/<c>mod_single</c>: width-exploiting refinement - bisects how
/// much projectile flight can be shaved while the target's earlier position
/// still sits laterally within the (angle-scaled) width, using a circle-circle
/// intersection for the aim point and picking the rear-side solution.</item>
/// </list>
/// The original's heuristics are kept on purpose: linear angle-to-width scaling
/// (a triangle peaking at full width when perpendicular), 1-unit slack
/// convergence, 20 bisection iterations, and the 1000-unit backward segment
/// extension. Fixed relative to the source: the undefined second return value
/// (`p2`) and missing range/degenerate-input guards.
/// </summary>
public sealed class GagongPredictionEngine
{
    private const double SegmentBackExtension = 1000.0; // mod: p10 extended 1000 units behind
    private const int BisectionIterations = 20;
    private const double SlackTolerance = 1.0;

    private readonly PredictionConfig _config;

    /// <summary>
    /// Creates a Gagong prediction engine with the specified configuration.
    /// </summary>
    public GagongPredictionEngine(PredictionConfig? config = null)
    {
        _config = config ?? PredictionConfig.Default;
    }

    /// <summary>
    /// Predicts using the Gagong algorithm. Projectile skillshots only.
    /// </summary>
    public PredictionResult PredictFromState(
        Skillshot skillshot,
        Point2D casterPosition,
        MovementState targetState,
        double hitboxRadius)
    {
        if (skillshot.GetProjectileSpeed() is not double projectileSpeed)
            return new PredictionResult.Unreachable("Gagong supports projectile skillshots only");

        var range = skillshot.GetMaxRange();
        var effectiveDelay = skillshot.GetDelay() + _config.NetworkCompensationDelay;
        var width = skillshot.GetEffectiveRadius(hitboxRadius); // full lateral reach (input.width)

        var targetPosition = targetState.GetPosition();
        var targetVelocity = targetState.GetVelocity();
        var targetSpeed = targetVelocity.Length;

        // Stationary target: near-edge aim, same convention as the main engine
        if (targetSpeed <= 1.0)
        {
            var distance = casterPosition.DistanceTo(targetPosition);
            if (distance > range)
                return new PredictionResult.OutOfRange(distance, range);

            var time = effectiveDelay + Math.Max(0, distance - width) / projectileSpeed;
            return new PredictionResult.Hit(
                time, targetPosition, targetPosition,
                ComputeConfidence(casterPosition, targetPosition, 0, projectileSpeed));
        }

        // Build the waypoint list: Pathing maps directly; straight-line movers get
        // a synthetic two-point path long enough to cover the prediction horizon
        IReadOnlyList<Point2D> waypoints;
        int index;
        Point2D serverPosition;

        if (targetState is MovementState.Pathing pathing && pathing.Waypoints.Count >= 1)
        {
            if (pathing.CurrentIndex >= pathing.Waypoints.Count)
            {
                // Finished path: stationary at the last waypoint
                return PredictFromState(
                    skillshot, casterPosition,
                    new MovementState.Idle(pathing.Waypoints[^1]), hitboxRadius);
            }

            waypoints = pathing.Waypoints;
            index = Math.Max(1, pathing.CurrentIndex); // Lua: path.index == 0 and 1 or path.index
            serverPosition = targetPosition;
        }
        else
        {
            var horizon = targetSpeed * (_config.MaxPredictionTime + 1.0);
            waypoints = new[] { targetPosition, targetPosition + targetVelocity.Normalize().ScaleBy(horizon) };
            index = 1;
            serverPosition = targetPosition;
        }

        var projected = Project(
            casterPosition, waypoints, serverPosition, index, effectiveDelay, projectileSpeed, targetSpeed);

        if (projected.EndOfPath)
        {
            // Lua fallback: cast at the last waypoint
            var endPoint = waypoints[^1];
            var endDistance = casterPosition.DistanceTo(endPoint);
            if (endDistance > range)
                return new PredictionResult.OutOfRange(endDistance, range);

            return new PredictionResult.Hit(
                effectiveDelay + endDistance / projectileSpeed, endPoint, endPoint,
                ComputeConfidence(casterPosition, endPoint, targetSpeed, projectileSpeed));
        }

        // Width-exploiting refinement (mod): pull the hit earlier along the path
        var (aimPoint, targetAt, flightDistance) = Refine(
            casterPosition, projected.Point, waypoints, projected.SegmentEndIndex,
            projectileSpeed, targetSpeed, width);

        var aimDistance = casterPosition.DistanceTo(aimPoint);
        if (aimDistance > range)
            return new PredictionResult.OutOfRange(aimDistance, range);

        return new PredictionResult.Hit(
            effectiveDelay + flightDistance / projectileSpeed,
            aimPoint,
            targetAt,
            ComputeConfidence(casterPosition, aimPoint, targetSpeed, projectileSpeed));
    }

    /// <summary>
    /// Port of <c>mathf.project</c>: solves |D + v·t| = v1·t for a target walking
    /// from segmentStart toward segmentEnd at speed v2. No delay or hitbox terms.
    /// </summary>
    private static (Point2D Point, double Time)? ProjectSegment(
        Point2D source, Point2D segmentStart, Point2D segmentEnd, double v1, double v2)
    {
        var k = segmentStart - source;
        var segment = segmentEnd - segmentStart;
        if (segment.Length < 1e-9)
            return null;

        var v = segment.Normalize().ScaleBy(v2);
        var a = v.DotProduct(v) - v1 * v1;
        var b = 2.0 * v.DotProduct(k);

        if (Math.Abs(a) > 1e-9)
        {
            var d = b * b - 4.0 * a * k.DotProduct(k);
            if (d > 0)
            {
                var t = (-b - Math.Sqrt(d)) / (2.0 * a);
                return (segmentStart + v.ScaleBy(t), t);
            }
        }
        else if (Math.Abs(b) > 1e-9)
        {
            var t = -k.DotProduct(k) / b;
            return (segmentStart + v.ScaleBy(t), t);
        }

        return null;
    }

    /// <summary>
    /// Port of <c>project</c>: walks the waypoints, handling the cast delay by
    /// advancing the first segment start (and extending later segment starts
    /// backward by the accumulated walked distance), so the per-segment quadratic
    /// needs no accumulated-time terms. Returns the interception point and the
    /// index of the segment-end waypoint, or the last waypoint with EndOfPath set.
    /// </summary>
    private static (Point2D Point, int SegmentEndIndex, bool EndOfPath) Project(
        Point2D source,
        IReadOnlyList<Point2D> waypoints,
        Point2D serverPosition,
        int index,
        double delay,
        double v1,
        double v2)
    {
        var cut = -v2 * delay; // negative accumulated distance (Lua: k = -v2*t)
        var minT = 0.0;

        // First leg: from the server position toward the current waypoint
        var length = serverPosition.DistanceTo(waypoints[index]);
        if (length > 1e-6)
        {
            var extended = Lerp(serverPosition, waypoints[index], -cut / length);
            var maxT = extended.DistanceTo(waypoints[index]) / v2;
            var result = ProjectSegment(source, extended, waypoints[index], v1, v2);
            if (result is var (point, time) && result is not null && time >= minT && time <= maxT)
                return (point, index, false);

            cut += length;
            minT = cut / v2;
        }

        // Remaining segments
        for (var i = index; i < waypoints.Count - 1; i++)
        {
            length = waypoints[i].DistanceTo(waypoints[i + 1]);
            if (length < 1e-6)
                continue;

            var extended = Lerp(waypoints[i], waypoints[i + 1], -cut / length);
            var maxT = extended.DistanceTo(waypoints[i + 1]) / v2;
            var result = ProjectSegment(source, extended, waypoints[i + 1], v1, v2);
            if (result is var (point, time) && result is not null && time >= minT && time <= maxT)
                return (point, i + 1, false);

            cut += length;
            minT = cut / v2;
        }

        return (waypoints[^1], waypoints.Count - 1, true);
    }

    /// <summary>
    /// Port of <c>mod</c>: bisects how much projectile flight can be shaved off
    /// (trial in [-width, 0]) while the width-graze construction stays feasible,
    /// converging to ~1 unit of slack. Returns the refined aim point, the
    /// target's position at that earlier contact, and the flight distance.
    /// </summary>
    private static (Point2D Aim, Point2D TargetAt, double FlightDistance) Refine(
        Point2D source,
        Point2D interceptPosition,
        IReadOnlyList<Point2D> waypoints,
        int segmentEndIndex,
        double v1,
        double v2,
        double width)
    {
        var flightDistance = source.DistanceTo(interceptPosition);
        var segmentStart = waypoints[segmentEndIndex - 1];
        var segmentEnd = waypoints[segmentEndIndex];
        var segmentLength = segmentStart.DistanceTo(segmentEnd);
        if (segmentLength < 1e-6)
            return (interceptPosition, interceptPosition, flightDistance);

        // Lua: p10 = p10:lerp(p11, -1000/len) - extend well behind the segment
        var extendedStart = Lerp(segmentStart, segmentEnd, -SegmentBackExtension / segmentLength);
        var d = interceptPosition.DistanceTo(extendedStart);
        if (d < 1e-6)
            return (interceptPosition, interceptPosition, flightDistance);

        var aim = interceptPosition;
        var targetAt = interceptPosition;
        var feasible = 0.0;
        var infeasible = -width;
        var slack = double.MaxValue;

        for (var i = 0; i < BisectionIterations && slack > SlackTolerance; i++)
        {
            var trial = (feasible + infeasible) * 0.5;
            var candidate = RefineSingle(
                source, interceptPosition, extendedStart, d, flightDistance, trial, v1, v2, width);

            if (candidate is var (point, earlierTarget, candidateSlack) && candidate is not null)
            {
                aim = point;
                targetAt = earlierTarget;
                feasible = trial;
                slack = candidateSlack;
            }
            else
            {
                infeasible = trial;
            }
        }

        return (aim, targetAt, source.DistanceTo(interceptPosition) + feasible);
    }

    /// <summary>
    /// Port of <c>mod_single</c>: tests whether shaving <paramref name="trial"/>
    /// units of projectile flight still hits. The target's earlier position l is
    /// found by walking back along the path; the aim point is the circle-circle
    /// intersection where the ray from the caster passes the target at lateral
    /// offset sqrt(r2), taking the rear-side solution; feasibility requires the
    /// offset to fit within the angle-scaled usable width.
    /// </summary>
    private static (Point2D Aim, Point2D TargetAt, double Slack)? RefineSingle(
        Point2D source,
        Point2D interceptPosition,
        Point2D extendedStart,
        double d,
        double interceptDistance,
        double trial,
        double v1,
        double v2,
        double width)
    {
        // l = target position if intercepted |trial| units of flight earlier
        var earlierTarget = Lerp(interceptPosition, extendedStart, -trial / v1 * v2 / d);
        var flight = interceptDistance + trial;
        if (flight <= 1e-6)
            return null;

        var offsetSq = source.DistanceTo(earlierTarget) * source.DistanceTo(earlierTarget) - flight * flight;
        if (offsetSq < 0 || offsetSq > width * width)
            return null;

        var lateralOffset = Math.Sqrt(offsetSq);
        if (CircleCircleIntersection(source, flight, earlierTarget, lateralOffset)
                is not var (candidate1, candidate2))
            return null;

        // Rear-side pick: the intersection closer to the (extended) segment start
        var aim = candidate1.DistanceTo(extendedStart) * candidate1.DistanceTo(extendedStart)
                < candidate2.DistanceTo(extendedStart) * candidate2.DistanceTo(extendedStart)
            ? candidate1
            : candidate2;

        if (LineLineIntersection(source, aim, extendedStart, earlierTarget) is not Point2D crossing)
            return null;

        // Angle-scaled usable width: a triangle peaking at full width when the
        // ray is perpendicular to the path (Lua's pa/halfPi scaling, verbatim)
        var toSource = source - crossing;
        var toTarget = earlierTarget - crossing;
        var angle = Math.Abs(Math.Atan2(
            toSource.X * toTarget.Y - toSource.Y * toTarget.X,
            toSource.DotProduct(toTarget)));

        var halfPi = Math.PI * 0.5;
        var usableWidth = angle <= halfPi
            ? angle / halfPi * width
            : (1 - (angle - halfPi) / halfPi) * width;

        var slack = usableWidth - lateralOffset;
        if (slack < usableWidth && slack > 0)
            return (aim, earlierTarget, slack);

        return null;
    }

    private static double ComputeConfidence(
        Point2D casterPosition, Point2D aimPoint, double targetSpeed, double projectileSpeed)
    {
        var distance = casterPosition.DistanceTo(aimPoint);
        var distanceFactor = Math.Max(0.5, 1.0 - distance / 2000.0);
        var speedFactor = Math.Max(0.6, 1.0 - targetSpeed / projectileSpeed);
        return Math.Min(1.0, distanceFactor * speedFactor);
    }

    private static Point2D Lerp(Point2D from, Point2D to, double fraction) =>
        from + (to - from).ScaleBy(fraction);

    private static (Point2D, Point2D)? CircleCircleIntersection(
        Point2D center0, double radius0, Point2D center1, double radius1)
    {
        var distance = center0.DistanceTo(center1);
        if (distance < 1e-9 || distance > radius0 + radius1 || distance < Math.Abs(radius0 - radius1))
            return null;

        var along = (radius0 * radius0 - radius1 * radius1 + distance * distance) / (2 * distance);
        var acrossSq = radius0 * radius0 - along * along;
        if (acrossSq < 0)
            return null;

        var across = Math.Sqrt(acrossSq);
        var direction = (center1 - center0).Normalize();
        var mid = center0 + direction.ScaleBy(along);
        var perpendicular = new Vector2D(-direction.Y, direction.X);

        return (mid + perpendicular.ScaleBy(across), mid - perpendicular.ScaleBy(across));
    }

    private static Point2D? LineLineIntersection(Point2D a1, Point2D a2, Point2D b1, Point2D b2)
    {
        var directionA = a2 - a1;
        var directionB = b2 - b1;
        var denominator = directionA.X * directionB.Y - directionA.Y * directionB.X;
        if (Math.Abs(denominator) < 1e-9)
            return null;

        var s = ((b1.X - a1.X) * directionB.Y - (b1.Y - a1.Y) * directionB.X) / denominator;
        return a1 + directionA.ScaleBy(s);
    }
}
