namespace Samaritan.Prediction.Engine;

using MathNet.Numerics;
using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Configuration;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;
using Samaritan.Prediction.Solvers;

/// <summary>
/// The pre-audit prediction algorithm, preserved verbatim from commit ffae73e
/// for A/B comparison against <see cref="PredictionEngine"/> (the simulation's
/// BEFORE mode). Only the result cache and timing stopwatch were omitted; the
/// calculation is byte-for-byte the original, including its known defects:
/// <list type="bullet">
/// <item>Instant skillshots use a double.MaxValue speed sentinel, which poisons
/// the waypoint quadratic with NaN (circular/cone vs pathing never hit).</item>
/// <item>The trailing-edge correction multiplier has a pole where
/// 2 - hitboxRatio - cosAngle/hitboxRatio crosses zero, producing infeasible
/// hits on one side and spurious Unreachable results on the other.</item>
/// <item>Pathing states whose CurrentIndex passed the last waypoint throw.</item>
/// </list>
/// Do not fix bugs in this class - it exists to show them.
/// </summary>
public sealed class LegacyPredictionEngine
{
    private readonly PredictionConfig _config;
    private readonly IInterceptionSolver _solver;

    /// <summary>
    /// Creates a legacy prediction engine with the specified configuration.
    /// </summary>
    public LegacyPredictionEngine(PredictionConfig? config = null)
    {
        _config = config ?? PredictionConfig.Default;
        _solver = new HybridSolver(_config);
    }

    /// <summary>
    /// Predicts using the original pre-audit algorithm.
    /// </summary>
    public PredictionResult PredictFromState(
        Skillshot skillshot,
        Point2D casterPosition,
        MovementState targetState,
        double hitboxRadius)
    {
        // Get skillshot parameters
        var (baseDelay, range) = GetSkillshotParams(skillshot);
        var skillshotSpeed = GetSkillshotSpeed(skillshot);

        // Add network compensation: ping + tick uncertainty + reaction buffer
        var effectiveDelay = baseDelay + _config.NetworkCompensationDelay;

        // Check basic range first
        var targetPosition = targetState.GetPosition();
        var distance = casterPosition.DistanceTo(targetPosition);

        if (distance > range * 1.5) // Allow some prediction leeway
        {
            return new PredictionResult.OutOfRange(distance, range);
        }

        // Get target movement info
        var targetVelocity = targetState.GetVelocity();
        var targetSpeed = targetVelocity.Length;
        var effectiveRadius = GetEffectiveRadius(skillshot, hitboxRadius);

        PredictionResult result;

        // For waypoint paths, use the specialized solver (handles direction changes)
        if (targetState is MovementState.Pathing pathing)
        {
            result = SolveWaypointInterception(
                skillshot, casterPosition, pathing, effectiveRadius, effectiveDelay, range);
        }
        // For traveling projectiles with moving targets, solve trailing edge interception directly
        else if (targetSpeed > 1.0 && skillshotSpeed < 10000)
        {
            var trailingEdgeResult = SolveTrailingEdgeInterception(
                casterPosition, targetPosition, targetVelocity, effectiveRadius,
                skillshotSpeed, effectiveDelay, range);

            if (trailingEdgeResult.HasValue)
            {
                var (interceptionTime, castPosition) = trailingEdgeResult.Value;
                var predictedPosition = targetState.PredictPosition(interceptionTime);

                result = new PredictionResult.Hit(
                    interceptionTime,
                    castPosition,
                    predictedPosition,
                    ComputeConfidence(casterPosition, castPosition, targetSpeed, skillshotSpeed));
            }
            else
            {
                result = new PredictionResult.Unreachable("No valid trailing edge interception found");
            }
        }
        else
        {
            // For stationary targets or instant skillshots, use the standard solver
            var solution = _solver.Solve(skillshot, casterPosition, targetState, hitboxRadius);

            if (solution is null)
            {
                result = new PredictionResult.Unreachable("No valid interception found");
            }
            else
            {
                // Add network compensation to the interception time
                var adjustedTime = solution.Value.Time + _config.NetworkCompensationDelay;
                var predictedPosition = targetState.PredictPosition(adjustedTime);
                var distanceToTarget = casterPosition.DistanceTo(predictedPosition);

                if (distanceToTarget > range)
                {
                    result = new PredictionResult.OutOfRange(distanceToTarget, range);
                }
                else
                {
                    // For stationary targets, aim at near edge
                    var castPosition = ComputeStaticAimPoint(
                        casterPosition, predictedPosition, effectiveRadius);

                    result = new PredictionResult.Hit(
                        adjustedTime,
                        castPosition,
                        predictedPosition,
                        solution.Value.Confidence);
                }
            }
        }

        return result;
    }

    /// <summary>
    /// Solves interception for targets following waypoint paths.
    /// Cuts the path by (delay * targetSpeed - hitbox), then solves using quadratic formula.
    /// </summary>
    private static PredictionResult SolveWaypointInterception(
        Skillshot skillshot,
        Point2D casterPosition,
        MovementState.Pathing pathing,
        double effectiveRadius,
        double effectiveDelay,
        double range)
    {
        var projectileSpeed = GetSkillshotSpeed(skillshot);
        var segments = pathing.GetPathSegments().ToList();

        if (segments.Count == 0)
            return new PredictionResult.Unreachable("No path segments");

        var targetSpeed = segments[0].Velocity.Length;

        // Cut path by (delay * speed - hitbox) to account for:
        // 1. Where target will be when projectile fires (delay * speed)
        // 2. Hitting the near edge of hitbox (-hitbox)
        var cutLength = effectiveDelay * targetSpeed - effectiveRadius;
        var cutSegments = CutPath(segments, cutLength);

        if (cutSegments.Count == 0)
            return new PredictionResult.Unreachable("Path cut resulted in empty path");

        // Quadratic formula (hitbox handled via path cutting)
        // a = v² - p², b = 2(diff·v - p²·tTotal), c = diff² - p²·tTotal²
        var sqrSpeed = projectileSpeed * projectileSpeed;
        double tTotal = 0;
        const double Epsilon = 1e-4;

        foreach (var segment in cutSegments)
        {
            var diff = segment.Start - casterPosition;
            var velocity = segment.Velocity;
            var duration = segment.Duration;

            // Quadratic coefficients
            var a = velocity.DotProduct(velocity) - sqrSpeed;
            var b = 2.0 * (diff.DotProduct(velocity) - sqrSpeed * tTotal);
            var c = diff.DotProduct(diff) - sqrSpeed * tTotal * tTotal;

            // Use FindRoots.Quadratic for robust numerical handling
            var (root1, root2) = FindRoots.Quadratic(c, b, a);

            const double ImagTol = 1e-9;
            var tIntercept = double.MaxValue;

            if (Math.Abs(root1.Imaginary) < ImagTol && root1.Real >= 0 && root1.Real <= duration + Epsilon)
                tIntercept = Math.Min(tIntercept, root1.Real);
            if (Math.Abs(root2.Imaginary) < ImagTol && root2.Real >= 0 && root2.Real <= duration + Epsilon)
                tIntercept = Math.Min(tIntercept, root2.Real);

            if (tIntercept < double.MaxValue)
            {
                // Aim point is simply position on the cut path at interception time
                var aimPoint = segment.Start + velocity.ScaleBy(tIntercept);

                if (casterPosition.DistanceTo(aimPoint) <= range)
                {
                    var totalTime = effectiveDelay + tTotal + tIntercept;
                    var predictedPosition = pathing.PredictPosition(totalTime);

                    return new PredictionResult.Hit(
                        totalTime,
                        aimPoint,
                        predictedPosition,
                        ComputeConfidence(casterPosition, aimPoint, pathing.Speed, projectileSpeed));
                }
            }

            tTotal += duration;
        }

        return new PredictionResult.Unreachable("No valid interception found on path");
    }

    /// <summary>
    /// Cuts a path by the specified distance.
    /// Positive distance advances along the path, negative distance extends backwards.
    /// </summary>
    private static List<PathSegment> CutPath(List<PathSegment> segments, double distance)
    {
        var result = new List<PathSegment>();

        if (segments.Count == 0)
            return result;

        if (distance < 0)
        {
            // Extend backwards
            var first = segments[0];
            var extendedStart = first.Start + first.Direction.ScaleBy(distance);
            var newStartTime = first.StartTime + distance / first.Speed;
            var newFirst = new PathSegment(extendedStart, first.End, newStartTime, first.EndTime, first.Speed);
            result.Add(newFirst);

            for (var i = 1; i < segments.Count; i++)
                result.Add(segments[i]);

            return result;
        }

        // Advance along path
        var remaining = distance;
        for (var i = 0; i < segments.Count; i++)
        {
            var seg = segments[i];
            var segLength = seg.Length;

            if (remaining < segLength)
            {
                var newStart = seg.Start + seg.Direction.ScaleBy(remaining);
                var newStartTime = seg.StartTime + remaining / seg.Speed;
                var newSegment = new PathSegment(newStart, seg.End, newStartTime, seg.EndTime, seg.Speed);
                result.Add(newSegment);

                for (var j = i + 1; j < segments.Count; j++)
                    result.Add(segments[j]);

                return result;
            }

            remaining -= segLength;
        }

        // Distance exceeds path - return last point as zero-length segment
        var last = segments[^1];
        result.Add(new PathSegment(last.End, last.End, last.EndTime, last.EndTime, last.Speed));
        return result;
    }

    /// <summary>
    /// Solves the interception problem for a moving target traveling in a straight line.
    /// Applies angle-dependent trailing edge correction for accurate hitbox handling.
    /// </summary>
    private static (double Time, Point2D AimPoint)? SolveTrailingEdgeInterception(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double hitbox,
        double projectileSpeed,
        double castDelay,
        double maxRange)
    {
        var targetSpeed = targetVelocity.Length;

        // Stationary target - aim at near edge
        if (targetSpeed < 1.0)
        {
            var toTarget = targetPosition - casterPosition;
            var distance = toTarget.Length;

            if (distance <= hitbox)
                return (castDelay, targetPosition);

            var flightTime = (distance - hitbox) / projectileSpeed;
            var nearEdge = targetPosition - toTarget.Normalize().ScaleBy(hitbox);

            if (casterPosition.DistanceTo(nearEdge) > maxRange)
                return null;

            return (castDelay + flightTime, nearEdge);
        }

        // Cut path by (delay * speed - hitbox) to get trailing edge start position
        var cutDistance = castDelay * targetSpeed - hitbox;
        var targetDirection = targetVelocity.Normalize();
        var trailingEdgeStart = targetPosition + targetDirection.ScaleBy(cutDistance);

        // Vector from caster to trailing edge start
        var toTrailingEdge = trailingEdgeStart - casterPosition;
        var distanceToTrailingEdge = toTrailingEdge.Length;
        var projectileSpeedSqr = projectileSpeed * projectileSpeed;

        // Angle between caster-to-target and target velocity
        var cosAngle = distanceToTrailingEdge > 1e-9
            ? toTrailingEdge.DotProduct(targetVelocity) / (distanceToTrailingEdge * targetSpeed)
            : 1.0;
        var sinAngle = Math.Sqrt(Math.Max(0, 1.0 - cosAngle * cosAngle));

        // Trailing edge correction factors
        var speedDifference = projectileSpeed - targetSpeed;
        var delayDistance = targetSpeed * castDelay;
        var extendedHitbox = hitbox + delayDistance;
        var baseCorrection = speedDifference * extendedHitbox;

        // Physically-derived correction multiplier using dimensionless ratios
        // M = sinθ × (1 + catchUpFactor × hitboxRatio / angleDivisor)
        var hitboxRatio = hitbox / extendedHitbox;
        var speedRatio = targetSpeed / projectileSpeed;
        var catchUpFactor = 1 - speedRatio;
        var angleDivisor = 2 - hitboxRatio - cosAngle / hitboxRatio;

        // Guard against division by zero (e.g. when hitboxRatio ~ 1 and cosAngle ~ 1)
        var correctionMultiplier = 0.0;
        if (Math.Abs(angleDivisor) > 1e-9)
        {
            correctionMultiplier = sinAngle * (1 + catchUpFactor * hitboxRatio / angleDivisor);
        }

        // Quadratic coefficients: at² + bt + c = 0
        var quadA = targetVelocity.DotProduct(targetVelocity) - projectileSpeedSqr;
        var quadB = 2.0 * (toTrailingEdge.DotProduct(targetVelocity) - baseCorrection * correctionMultiplier);
        var quadC = toTrailingEdge.DotProduct(toTrailingEdge);

        // Solve quadratic using robust numerical method
        var (root1, root2) = FindRoots.Quadratic(quadC, quadB, quadA);

        // Find minimum valid real root
        const double ImaginaryTolerance = 1e-9;
        var maxFlightTime = maxRange / projectileSpeed;
        var interceptTime = double.MaxValue;

        if (Math.Abs(root1.Imaginary) < ImaginaryTolerance && root1.Real >= 0 && root1.Real <= maxFlightTime)
            interceptTime = Math.Min(interceptTime, root1.Real);
        if (Math.Abs(root2.Imaginary) < ImaginaryTolerance && root2.Real >= 0 && root2.Real <= maxFlightTime)
            interceptTime = Math.Min(interceptTime, root2.Real);

        if (interceptTime >= double.MaxValue)
            return null;

        var totalTime = castDelay + interceptTime;
        var aimPoint = trailingEdgeStart + targetVelocity.ScaleBy(interceptTime);

        if (casterPosition.DistanceTo(aimPoint) > maxRange)
            return null;

        return (totalTime, aimPoint);
    }

    /// <summary>
    /// Computes confidence based on the difficulty of the shot.
    /// </summary>
    private static double ComputeConfidence(
        Point2D casterPosition,
        Point2D aimPoint,
        double targetSpeed,
        double projectileSpeed)
    {
        var distance = casterPosition.DistanceTo(aimPoint);

        // Base confidence decreases with distance
        var distanceFactor = Math.Max(0.5, 1.0 - distance / 2000.0);

        // Confidence decreases when target is fast relative to projectile
        var speedRatio = targetSpeed / projectileSpeed;
        var speedFactor = Math.Max(0.6, 1.0 - speedRatio);

        return Math.Min(1.0, distanceFactor * speedFactor);
    }

    /// <summary>
    /// For stationary targets, aim at the near edge (toward caster).
    /// </summary>
    private static Point2D ComputeStaticAimPoint(
        Point2D casterPosition,
        Point2D targetPosition,
        double effectiveRadius)
    {
        var distance = casterPosition.DistanceTo(targetPosition);
        if (distance <= effectiveRadius)
            return targetPosition;

        var directionToTarget = (targetPosition - casterPosition).Normalize();
        return new Point2D(
            targetPosition.X - directionToTarget.X * effectiveRadius,
            targetPosition.Y - directionToTarget.Y * effectiveRadius);
    }

    private static (double Delay, double Range) GetSkillshotParams(Skillshot skillshot)
    {
        return skillshot.Match(
            linear: l => ((double)l.Delay, (double)l.Range),
            circular: c => ((double)c.Delay, (double)c.Range),
            cone: c => ((double)c.Delay, (double)c.Range),
            arc: a => ((double)a.Delay, (double)a.OuterRadius),
            rectangle: r => ((double)r.Delay, (double)r.Range),
            vectorRectangle: v => ((double)v.Delay, (double)(v.Range + v.MaxLength)));
    }

    /// <summary>
    /// Gets the projectile speed for a skillshot. Returns a high value for instant skillshots.
    /// </summary>
    private static double GetSkillshotSpeed(Skillshot skillshot)
    {
        return skillshot.Match(
            linear: l => (double)l.Speed,
            circular: _ => double.MaxValue, // Instant cast (no projectile travel time)
            cone: _ => double.MaxValue,     // Instant cast (no projectile travel time)
            arc: a => (double)a.Speed,
            rectangle: r => (double)r.Speed,
            vectorRectangle: v => (double)v.Speed);
    }

    /// <summary>
    /// Gets the effective hit radius for a skillshot type.
    /// This is the distance from target center at which a hit occurs.
    /// </summary>
    private static double GetEffectiveRadius(Skillshot skillshot, double hitboxRadius)
    {
        return skillshot.Match(
            linear: l => l.Width / 2.0 + hitboxRadius,
            circular: c => c.Radius + hitboxRadius,
            cone: _ => hitboxRadius, // Cone has no width (instant area-of-effect from point)
            arc: a => a.Width / 2.0 + hitboxRadius,
            rectangle: r => r.Width / 2.0 + hitboxRadius,
            vectorRectangle: v => v.Width / 2.0 + hitboxRadius);
    }
}
