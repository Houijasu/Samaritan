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
///
/// Performance notes (behavior-identical, pinned by golden-value tests):
/// straight-line movers take an allocation-free fast path; the refinement loop
/// runs on scalars with the circle-circle intersection reduced via the
/// right-angle property of the tangent-length construction (along = f²/D,
/// across = f·lateral/D), cutting ~7 square roots per iteration down to 2.
/// </summary>
public sealed class GagongPredictionEngine
{
    private const double SegmentBackExtension = 1000.0; // mod: p10 extended 1000 units behind
    private const int BisectionIterations = 20;
    private const double SlackTolerance = 1.0;
    private const double HalfPi = Math.PI * 0.5;

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
        // Single dispatch for all skillshot parameters (same values as the
        // GetDelay/GetProjectileSpeed/GetMaxRange/GetEffectiveRadius extensions)
        var (delay, projectileSpeed, range, halfWidth) = skillshot.Match(
            linear: l => ((double)l.Delay, (double)l.Speed, (double)l.Range, l.Width / 2.0),
            circular: c => ((double)c.Delay, (double)c.Speed, (double)c.Range, (double)c.Radius),
            cone: c => ((double)c.Delay, 0.0, (double)c.Range, 0.0),
            arc: a => ((double)a.Delay, (double)a.Speed, (double)a.OuterRadius, a.Width / 2.0),
            rectangle: r => ((double)r.Delay, (double)r.Speed, (double)r.Range, r.Width / 2.0),
            vectorRectangle: v => ((double)v.Delay, (double)v.Speed, (double)(v.Range + v.MaxLength), v.Width / 2.0));

        if (projectileSpeed <= 0)
            return new PredictionResult.Unreachable("Gagong supports projectile skillshots only");

        var effectiveDelay = delay + _config.NetworkCompensationDelay;
        var width = halfWidth + hitboxRadius; // full lateral reach (input.width)

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

        Point2D interceptPosition;
        Point2D segmentStart;
        Point2D segmentEnd;

        if (targetState is MovementState.Pathing pathing && pathing.Waypoints.Count >= 1)
        {
            if (pathing.CurrentIndex >= pathing.Waypoints.Count)
            {
                // Finished path: stationary at the last waypoint
                return PredictFromState(
                    skillshot, casterPosition,
                    new MovementState.Idle(pathing.Waypoints[^1]), hitboxRadius);
            }

            var waypoints = pathing.Waypoints;
            var index = Math.Max(1, pathing.CurrentIndex); // Lua: path.index == 0 and 1 or path.index

            var projected = Project(
                casterPosition, waypoints, targetPosition, index, effectiveDelay,
                projectileSpeed, targetSpeed);

            if (projected.EndOfPath)
                return EndOfPathFallback(
                    casterPosition, waypoints[^1], range, effectiveDelay, projectileSpeed, targetSpeed);

            interceptPosition = projected.Point;
            segmentStart = waypoints[projected.SegmentEndIndex - 1];
            segmentEnd = waypoints[projected.SegmentEndIndex];
        }
        else
        {
            // Straight-line fast path: equivalent to the synthetic two-point
            // waypoint walk, with no array, list dispatch, or normalization.
            // Virtual start = position at launch; v = the velocity itself.
            var launch = targetPosition + targetVelocity.ScaleBy(effectiveDelay);
            var kx = launch.X - casterPosition.X;
            var ky = launch.Y - casterPosition.Y;
            var vx = targetVelocity.X;
            var vy = targetVelocity.Y;

            var a = targetSpeed * targetSpeed - projectileSpeed * projectileSpeed;
            var b = 2.0 * (vx * kx + vy * ky);
            var maxT = _config.MaxPredictionTime + 1.0 - effectiveDelay;

            double? interceptTime = null;
            if (Math.Abs(a) > 1e-9)
            {
                var discriminant = b * b - 4.0 * a * (kx * kx + ky * ky);
                if (discriminant > 0)
                    interceptTime = (-b - Math.Sqrt(discriminant)) / (2.0 * a);
            }
            else if (Math.Abs(b) > 1e-9)
            {
                interceptTime = -(kx * kx + ky * ky) / b;
            }

            if (interceptTime is not double t || t < 0 || t > maxT)
            {
                // End of the synthetic horizon: cast at the far point
                var horizonEnd = targetPosition + targetVelocity.ScaleBy(_config.MaxPredictionTime + 1.0);
                return EndOfPathFallback(
                    casterPosition, horizonEnd, range, effectiveDelay, projectileSpeed, targetSpeed);
            }

            interceptPosition = launch + targetVelocity.ScaleBy(t);
            segmentStart = targetPosition;
            segmentEnd = targetPosition + targetVelocity.ScaleBy(_config.MaxPredictionTime + 1.0);
        }

        // Width-exploiting refinement (mod): pull the hit earlier along the path
        var (aimPoint, targetAt, flightDistance) = Refine(
            casterPosition, interceptPosition, segmentStart, segmentEnd,
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

    private static PredictionResult EndOfPathFallback(
        Point2D casterPosition,
        Point2D endPoint,
        double range,
        double effectiveDelay,
        double projectileSpeed,
        double targetSpeed)
    {
        var endDistance = casterPosition.DistanceTo(endPoint);
        if (endDistance > range)
            return new PredictionResult.OutOfRange(endDistance, range);

        return new PredictionResult.Hit(
            effectiveDelay + endDistance / projectileSpeed, endPoint, endPoint,
            ComputeConfidence(casterPosition, endPoint, targetSpeed, projectileSpeed));
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
    /// Port of <c>mod</c>/<c>mod_single</c>: bisects how much projectile flight
    /// can be shaved off (trial in [-width, 0]) while the width-graze
    /// construction stays feasible, converging to ~1 unit of slack. The loop
    /// body runs on scalars; the circle-circle intersection is reduced through
    /// the right-angle property of the tangent-length construction
    /// (along = flight²/D, across = flight·lateral/D - algebraically identical
    /// to the general intersection for this configuration).
    /// </summary>
    private static (Point2D Aim, Point2D TargetAt, double FlightDistance) Refine(
        Point2D source,
        Point2D interceptPosition,
        Point2D segmentStart,
        Point2D segmentEnd,
        double v1,
        double v2,
        double width)
    {
        var baseFlight = source.DistanceTo(interceptPosition);
        var segmentLength = segmentStart.DistanceTo(segmentEnd);
        if (segmentLength < 1e-6)
            return (interceptPosition, interceptPosition, baseFlight);

        // Lua: p10 = p10:lerp(p11, -1000/len) - extend well behind the segment
        var extendedStart = Lerp(segmentStart, segmentEnd, -SegmentBackExtension / segmentLength);
        var ex = extendedStart.X;
        var ey = extendedStart.Y;

        var sx = source.X;
        var sy = source.Y;
        var ix = interceptPosition.X;
        var iy = interceptPosition.Y;

        var dx = ex - ix;
        var dy = ey - iy;
        var d = Math.Sqrt(dx * dx + dy * dy);
        if (d < 1e-6)
            return (interceptPosition, interceptPosition, baseFlight);

        // Unit back-direction along the path (toward the extended start)
        var ubx = dx / d;
        var uby = dy / d;
        var widthSq = width * width;

        var aim = interceptPosition;
        var targetAt = interceptPosition;
        var feasible = 0.0;
        var infeasible = -width;
        var slack = double.MaxValue;

        for (var i = 0; i < BisectionIterations && slack > SlackTolerance; i++)
        {
            var trial = (feasible + infeasible) * 0.5;

            // l = target position if intercepted |trial| units of flight earlier
            var back = -trial * v2 / v1;
            var lx = ix + ubx * back;
            var ly = iy + uby * back;

            var flight = baseFlight + trial;
            if (flight <= 1e-6)
            {
                infeasible = trial;
                continue;
            }

            var dlx = lx - sx;
            var dly = ly - sy;
            var distSq = dlx * dlx + dly * dly;
            var offsetSq = distSq - flight * flight;
            if (offsetSq < 0 || offsetSq > widthSq)
            {
                infeasible = trial;
                continue;
            }

            var lateral = Math.Sqrt(offsetSq);
            var centerDistance = Math.Sqrt(distSq);
            if (centerDistance < 1e-9)
            {
                infeasible = trial;
                continue;
            }

            // Reduced circle-circle: |X - source| = flight, |X - l| = lateral,
            // right angle at X => along = flight²/D, across = flight·lateral/D
            var along = flight * flight / centerDistance;
            var across = flight * lateral / centerDistance;
            var invD = 1.0 / centerDistance;
            var mx = sx + dlx * along * invD;
            var my = sy + dly * along * invD;
            var px = -dly * invD;
            var py = dlx * invD;

            var c1x = mx + px * across;
            var c1y = my + py * across;
            var c2x = mx - px * across;
            var c2y = my - py * across;

            // Rear-side pick: the intersection closer to the (extended) segment start
            var d1x = c1x - ex;
            var d1y = c1y - ey;
            var d2x = c2x - ex;
            var d2y = c2y - ey;
            double cx, cy;
            if (d1x * d1x + d1y * d1y < d2x * d2x + d2y * d2y)
            {
                cx = c1x;
                cy = c1y;
            }
            else
            {
                cx = c2x;
                cy = c2y;
            }

            // Intersection of the aim ray (source -> c) with the path line
            var rax = cx - sx;
            var ray = cy - sy;
            var rbx = lx - ex;
            var rby = ly - ey;
            var denominator = rax * rby - ray * rbx;
            if (Math.Abs(denominator) < 1e-9)
            {
                infeasible = trial;
                continue;
            }

            var s = ((ex - sx) * rby - (ey - sy) * rbx) / denominator;
            var cpx = sx + rax * s;
            var cpy = sy + ray * s;

            // Angle-scaled usable width: a triangle peaking at full width when the
            // ray is perpendicular to the path (Lua's pa/halfPi scaling, verbatim)
            var ux = sx - cpx;
            var uy = sy - cpy;
            var wx = lx - cpx;
            var wy = ly - cpy;
            var angle = Math.Abs(Math.Atan2(ux * wy - uy * wx, ux * wx + uy * wy));

            var usableWidth = angle <= HalfPi
                ? angle / HalfPi * width
                : (1 - (angle - HalfPi) / HalfPi) * width;

            var candidateSlack = usableWidth - lateral;
            if (candidateSlack < usableWidth && candidateSlack > 0)
            {
                aim = new Point2D(cx, cy);
                targetAt = new Point2D(lx, ly);
                feasible = trial;
                slack = candidateSlack;
            }
            else
            {
                infeasible = trial;
            }
        }

        return (aim, targetAt, baseFlight + feasible);
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
}
