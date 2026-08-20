namespace Samaritan.Prediction.Engine;

using MathNet.Numerics;
using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Collision;
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
/// Implemented on MathNet.Spatial primitives (Point2D / Vector2D / Line2D) and
/// MathNet.Numerics (FindRoots) throughout, while keeping the fast structure
/// (behavior pinned by golden-value tests): straight-line movers take an
/// allocation-free fast path, and the refinement loop uses the circle-circle
/// intersection reduced via the right-angle property of the tangent-length
/// construction (along = f²/D, across = f·lateral/D).
/// </summary>
public sealed class GagongPredictionEngine : IPredictionEngine
{
    private const double SegmentBackExtension = 1000.0; // mod: p10 extended 1000 units behind
    private const int BisectionIterations = 20;
    private const double SlackTolerance = 1.0;
    private const double HalfPi = Math.PI * 0.5;

    private readonly PredictionConfig _config;
    private readonly CollisionValidationService _collisionService = new();

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
    /// <param name="aimMode">
    /// Ignored: Gagong is a faithful port of a single fixed algorithm and has
    /// no aim modes. Accepted only to satisfy <see cref="IPredictionEngine"/>.
    /// </param>
    public PredictionResult PredictFromState(
        Skillshot skillshot,
        Point2D casterPosition,
        MovementState targetState,
        double hitboxRadius,
        ProjectileAimMode aimMode = ProjectileAimMode.RearGraze)
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
            // waypoint walk, with no array or list dispatch. Virtual start =
            // position at launch; the segment velocity is the velocity itself.
            var launch = targetPosition + targetVelocity.ScaleBy(effectiveDelay);
            var launchOffset = launch - casterPosition;

            var quadA = targetVelocity.DotProduct(targetVelocity) - projectileSpeed * projectileSpeed;
            var quadB = 2.0 * targetVelocity.DotProduct(launchOffset);
            var quadC = launchOffset.DotProduct(launchOffset);
            var maxT = _config.MaxPredictionTime + 1.0 - effectiveDelay;

            if (SolveGagongQuadratic(quadA, quadB, quadC) is not double interceptTime
                || interceptTime < 0
                || interceptTime > maxT)
            {
                // End of the synthetic horizon: cast at the far point
                var horizonEnd = targetPosition + targetVelocity.ScaleBy(_config.MaxPredictionTime + 1.0);
                return EndOfPathFallback(
                    casterPosition, horizonEnd, range, effectiveDelay, projectileSpeed, targetSpeed);
            }

            interceptPosition = launch + targetVelocity.ScaleBy(interceptTime);
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

    /// <inheritdoc />
    public PredictionResult Predict(
        Skillshot skillshot,
        Point2D casterPosition,
        MovementTracker target)
    {
        var state = target.CurrentState;
        var hitboxRadius = target.HitboxRadius > 0 ? target.HitboxRadius : _config.DefaultHitboxRadius;

        return PredictFromState(skillshot, casterPosition, state, hitboxRadius);
    }

    /// <inheritdoc />
    public IReadOnlyList<PredictionResult> PredictMultiple(
        Skillshot skillshot,
        Point2D casterPosition,
        IEnumerable<MovementTracker> targets)
    {
        return targets
            .Select(t => Predict(skillshot, casterPosition, t))
            .ToList();
    }

    /// <inheritdoc />
    public bool ValidateHit(
        Skillshot skillshot,
        Point2D casterPosition,
        Point2D aimPosition,
        Point2D targetPosition,
        double hitboxRadius,
        double timeElapsed)
    {
        return _collisionService.ValidateHit(
            skillshot,
            casterPosition,
            aimPosition,
            targetPosition,
            hitboxRadius,
            timeElapsed);
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
    /// Solves a·t² + b·t + c = 0 with the Lua port's root choice
    /// (-b - sqrt(d))/(2a) via MathNet.Numerics: the larger real root when the
    /// target is slower than the projectile (a &lt; 0), the smaller one when
    /// faster. The original requires a strictly positive discriminant; the
    /// degenerate a ≈ 0 case (equal speeds) collapses to the linear solution.
    /// </summary>
    private static double? SolveGagongQuadratic(double a, double b, double c)
    {
        if (Math.Abs(a) > 1e-9)
        {
            if (b * b - 4.0 * a * c <= 0)
                return null;

            var (root1, root2) = FindRoots.Quadratic(c, b, a);
            return a < 0
                ? Math.Max(root1.Real, root2.Real)
                : Math.Min(root1.Real, root2.Real);
        }

        if (Math.Abs(b) > 1e-9)
            return -c / b;

        return null;
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
        var quadA = v.DotProduct(v) - v1 * v1;
        var quadB = 2.0 * v.DotProduct(k);
        var quadC = k.DotProduct(k);

        if (SolveGagongQuadratic(quadA, quadB, quadC) is not double t)
            return null;

        return (segmentStart + v.ScaleBy(t), t);
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
    /// construction stays feasible, converging to ~1 unit of slack. The
    /// circle-circle intersection is reduced through the right-angle property of
    /// the tangent-length construction (along = flight²/D, across =
    /// flight·lateral/D - algebraically identical to the general intersection
    /// for this configuration); the ray/path crossing uses MathNet's Line2D.
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
        var toExtended = extendedStart - interceptPosition;
        var extendedDistance = toExtended.Length;
        if (extendedDistance < 1e-6)
            return (interceptPosition, interceptPosition, baseFlight);

        // Unit back-direction along the path (toward the extended start)
        var backDirection = toExtended.ScaleBy(1.0 / extendedDistance);
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
            var earlierTarget = interceptPosition + backDirection.ScaleBy(-trial * v2 / v1);
            var flight = baseFlight + trial;
            if (flight <= 1e-6)
            {
                infeasible = trial;
                continue;
            }

            var toTarget = earlierTarget - source;
            var distanceSq = toTarget.DotProduct(toTarget);
            var offsetSq = distanceSq - flight * flight;
            if (offsetSq < 0 || offsetSq > widthSq)
            {
                infeasible = trial;
                continue;
            }

            var lateral = Math.Sqrt(offsetSq);
            var centerDistance = Math.Sqrt(distanceSq);
            if (centerDistance < 1e-9)
            {
                infeasible = trial;
                continue;
            }

            // Reduced circle-circle: |X - source| = flight, |X - l| = lateral,
            // right angle at X => along = flight²/D, across = flight·lateral/D
            var direction = toTarget.ScaleBy(1.0 / centerDistance);
            var perpendicular = new Vector2D(-direction.Y, direction.X);
            var mid = source + direction.ScaleBy(flight * flight / centerDistance);
            var chord = perpendicular.ScaleBy(flight * lateral / centerDistance);

            var candidate1 = mid + chord;
            var candidate2 = mid - chord;

            // Rear-side pick: the intersection closer to the (extended) segment start
            var offset1 = candidate1 - extendedStart;
            var offset2 = candidate2 - extendedStart;
            var aimCandidate = offset1.DotProduct(offset1) < offset2.DotProduct(offset2)
                ? candidate1
                : candidate2;

            // Intersection of the aim ray with the path line (infinite lines)
            if (source.DistanceTo(aimCandidate) < 1e-9 || extendedStart.DistanceTo(earlierTarget) < 1e-9)
            {
                infeasible = trial;
                continue;
            }

            var crossing = new Line2D(source, aimCandidate)
                .IntersectWith(new Line2D(extendedStart, earlierTarget));
            if (crossing is not Point2D crossingPoint)
            {
                infeasible = trial;
                continue;
            }

            // Angle-scaled usable width: a triangle peaking at full width when the
            // ray is perpendicular to the path (Lua's pa/halfPi scaling, verbatim)
            var toSource = source - crossingPoint;
            var toEarlier = earlierTarget - crossingPoint;
            var angle = Math.Abs(Math.Atan2(
                toSource.CrossProduct(toEarlier), toSource.DotProduct(toEarlier)));

            var usableWidth = angle <= HalfPi
                ? angle / HalfPi * width
                : (1 - (angle - HalfPi) / HalfPi) * width;

            var candidateSlack = usableWidth - lateral;
            if (candidateSlack < usableWidth && candidateSlack > 0)
            {
                aim = aimCandidate;
                targetAt = earlierTarget;
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
