namespace Samaritan.Prediction.Engine;

using System.Diagnostics;

using MathNet.Numerics;
using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Caching;
using Samaritan.Prediction.Collision;
using Samaritan.Prediction.Configuration;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;
using Samaritan.Prediction.Solvers;

/// <summary>
/// Main prediction engine that coordinates solvers and caching.
/// </summary>
public sealed class PredictionEngine : IPredictionEngine
{
    private PredictionConfig _config;
    private readonly IInterceptionSolver _solver;
    private readonly PredictionCache? _cache;
    private readonly CollisionValidationService _collisionService;

    /// <summary>
    /// Creates a prediction engine with the specified configuration.
    /// </summary>
    /// <param name="config">Prediction configuration.</param>
    /// <param name="enableCaching">Whether to enable prediction caching.</param>
    public PredictionEngine(PredictionConfig? config = null, bool enableCaching = true)
    {
        _config = config ?? PredictionConfig.Default;
        _solver = new HybridSolver(_config);
        _cache = enableCaching ? new PredictionCache(_config.CacheCapacity, _config.CacheTtlMs) : null;
        _collisionService = new CollisionValidationService();
    }

    /// <summary>
    /// Creates a prediction engine with a custom solver.
    /// </summary>
    public PredictionEngine(IInterceptionSolver solver, PredictionConfig? config = null, bool enableCaching = true)
    {
        _config = config ?? PredictionConfig.Default;
        _solver = solver;
        _cache = enableCaching ? new PredictionCache(_config.CacheCapacity, _config.CacheTtlMs) : null;
        _collisionService = new CollisionValidationService();
    }

    /// <summary>
    /// Updates the network ping for predictions.
    /// Call this periodically with the current game ping to maintain accuracy.
    /// </summary>
    /// <param name="pingMs">Current ping in milliseconds.</param>
    public void UpdatePing(double pingMs)
    {
        _config = _config with { PingMs = pingMs };
        _cache?.Clear(); // Invalidate cache since predictions change with new ping
    }

    /// <summary>
    /// Updates network settings for predictions.
    /// </summary>
    /// <param name="pingMs">Current ping in milliseconds.</param>
    /// <param name="reactionBufferMs">Optional reaction time buffer in milliseconds.</param>
    public void UpdateNetworkSettings(double pingMs, double? reactionBufferMs = null)
    {
        _config = reactionBufferMs.HasValue
        ? _config with { PingMs = pingMs, ReactionBufferMs = reactionBufferMs.Value }
        : _config with { PingMs = pingMs };
        _cache?.Clear();
    }

    /// <summary>
    /// Gets the current network compensation delay being used.
    /// </summary>
    public double CurrentNetworkCompensation => _config.NetworkCompensationDelay;

    /// <summary>
    /// Gets the current ping setting in milliseconds.
    /// </summary>
    public double CurrentPingMs => _config.PingMs;

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
    public PredictionResult PredictFromState(
        Skillshot skillshot,
        Point2D casterPosition,
        MovementState targetState,
        double hitboxRadius)
    {
        var sw = Stopwatch.StartNew();

        // Check cache
        var cacheKey = CreateCacheKey(skillshot, casterPosition, targetState);
        if (_cache?.TryGet(cacheKey, out var cachedResult) == true)
        {
            return cachedResult;
        }

        // Get skillshot parameters
        var (baseDelay, range) = GetSkillshotParams(skillshot);
        var skillshotSpeed = GetSkillshotSpeed(skillshot);

        // Add network compensation: ping + tick uncertainty + reaction buffer
        // This accounts for the time between seeing the target and the server processing your cast
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
            // Use full effectiveRadius - the new equation handles edge-to-edge collision properly
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

        sw.Stop();

        // Cache result
        _cache?.Set(cacheKey, result);

        return result;
    }

    /// <summary>
    /// Solves interception for targets following waypoint paths using the clean approach.
    /// Cuts the path by (delay * targetSpeed - hitbox), then uses clean quadratic.
    /// </summary>
    private PredictionResult SolveWaypointInterception(
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
        // Reference: SFML missile interception demo
        var cutLength = effectiveDelay * targetSpeed - effectiveRadius;
        var cutSegments = CutPath(segments, cutLength);

        if (cutSegments.Count == 0)
            return new PredictionResult.Unreachable("Path cut resulted in empty path");

        // Clean quadratic formula (hitbox handled via path cutting)
        // a = v² - p², b = 2(diff·v - p²·tTotal), c = diff² - p²·tTotal²
        var sqrSpeed = projectileSpeed * projectileSpeed;
        double tTotal = 0;
        const double Epsilon = 1e-4;

        foreach (var segment in cutSegments)
        {
            var diff = segment.Start - casterPosition;
            var velocity = segment.Velocity;
            var duration = segment.Duration;

            // Clean quadratic coefficients
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
    /// Cuts a path by the specified distance. Positive advances, negative extends backwards.
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
    /// Solves the interception problem for a moving target (single segment).
    /// Uses path cutting for hitbox, then applies angle-dependent trailing edge correction.
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
        var cosAngle = toTrailingEdge.DotProduct(targetVelocity) / (distanceToTrailingEdge * targetSpeed);
        var sinAngle = Math.Sqrt(1.0 - cosAngle * cosAngle);

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
        var correctionMultiplier = sinAngle * (1 + catchUpFactor * hitboxRatio / angleDivisor);

        // Quadratic coefficients: at² + bt + c = 0
        var quadA = targetVelocity.DotProduct(targetVelocity) - projectileSpeedSqr;
        var quadB = 2.0 * (toTrailingEdge.DotProduct(targetVelocity) - baseCorrection * correctionMultiplier);
        var quadC = toTrailingEdge.DotProduct(toTrailingEdge);

        // Solve quadratic using robust numerical method
        var (root1, root2) = FindRoots.Quadratic(quadC, quadB, quadA);

        // Find minimum valid real root
        const double imaginaryTolerance = 1e-9;
        var maxFlightTime = maxRange / projectileSpeed;
        var interceptTime = double.MaxValue;

        if (Math.Abs(root1.Imaginary) < imaginaryTolerance && root1.Real >= 0 && root1.Real <= maxFlightTime)
            interceptTime = Math.Min(interceptTime, root1.Real);
        if (Math.Abs(root2.Imaginary) < imaginaryTolerance && root2.Real >= 0 && root2.Real <= maxFlightTime)
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
    /// Predicts interception using the exact analytical method (Effective Delay).
    /// Used for comparison with the trailing edge approximation.
    /// </summary>
    public PredictionResult PredictExact(
        Skillshot skillshot,
        Point2D casterPosition,
        MovementState targetState,
        double hitboxRadius)
    {
        // Get skillshot parameters
        var (baseDelay, range) = GetSkillshotParams(skillshot);
        var skillshotSpeed = GetSkillshotSpeed(skillshot);
        var effectiveDelay = baseDelay + _config.NetworkCompensationDelay;

        // Get target info
        var targetPosition = targetState.GetPosition();
        var targetVelocity = targetState.GetVelocity();
        var targetSpeed = targetVelocity.Length;
        var effectiveRadius = GetEffectiveRadius(skillshot, hitboxRadius);

        // 1. Calculate Effective Delay: d' = d - R/s
        // This effectively launches the projectile earlier because it only needs to reach the edge
        var reducedDelay = effectiveDelay - effectiveRadius / skillshotSpeed;

        // 2. Solve standard quadratic with reduced delay
        var diff = targetPosition - casterPosition;
        var sqrSpeed = skillshotSpeed * skillshotSpeed;

        // a = v^2 - s^2
        var a = targetVelocity.DotProduct(targetVelocity) - sqrSpeed;

        // b = 2(D.V + s^2 * d')
        var b = 2.0 * (diff.DotProduct(targetVelocity) + sqrSpeed * reducedDelay);

        // c = D^2 - s^2 * d'^2
        var c = diff.DotProduct(diff) - sqrSpeed * reducedDelay * reducedDelay;

        // Use FindRoots.Quadratic
        var (root1, root2) = FindRoots.Quadratic(c, b, a);

        const double ImagTol = 1e-9;
        var tIntercept = double.MaxValue;

        // The interception time T must be greater than the launch time (effectiveDelay)
        // However, with reducedDelay, we are solving for the time the center reaches the target center.
        // Wait, the equation |P+VT - C| = s(T - d') solves for when the projectile (started at d') hits the target.
        // Since d' < d, and we want physical validity:
        // The projectile physically launches at 'd'.
        // Impact happens at T.
        // Flight time = T - d.
        // Distance = s * (T - d).
        // Condition: |P(T) - C| = s * (T - d) + R
        // |P+VT - C| - R = s(T - d)
        // This is not exactly quadratic unless we square it.
        // (|P+VT-C| - R)^2 = s^2 (T-d)^2
        // This is hard to solve.

        // My proposed "Effective Delay" shortcut:
        // Assume |P+VT - C| approx = s(T - d) + R
        // => |P+VT - C| = s(T - d + R/s) = s(T - (d - R/s))
        // So yes, d' = d - R/s.
        // The validity condition is T >= d (projectile must be launched).

        if (Math.Abs(root1.Imaginary) < ImagTol && root1.Real >= effectiveDelay)
            tIntercept = Math.Min(tIntercept, root1.Real);
        if (Math.Abs(root2.Imaginary) < ImagTol && root2.Real >= effectiveDelay)
            tIntercept = Math.Min(tIntercept, root2.Real);

        if (tIntercept >= double.MaxValue)
            return new PredictionResult.Unreachable("No valid exact interception found");

        var aimPoint = targetPosition + targetVelocity.ScaleBy(tIntercept);

        if (casterPosition.DistanceTo(aimPoint) > range + effectiveRadius)
            return new PredictionResult.OutOfRange(casterPosition.DistanceTo(aimPoint), range);

        var predictedPosition = targetState.PredictPosition(tIntercept);

        return new PredictionResult.Hit(
            tIntercept,
            aimPoint,
            predictedPosition,
            ComputeConfidence(casterPosition, aimPoint, targetSpeed, skillshotSpeed));
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
            circular: _ => double.MaxValue, // Instant
            cone: _ => double.MaxValue,     // Instant
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
            cone: _ => hitboxRadius, // Cone is instant area effect
            arc: a => a.Width / 2.0 + hitboxRadius,
            rectangle: r => r.Width / 2.0 + hitboxRadius,
            vectorRectangle: v => v.Width / 2.0 + hitboxRadius);
    }

    /// <summary>
    /// Gets the skillshot's width (without hitbox radius).
    /// Used for trailing edge offset to ensure visual contact.
    /// </summary>
    private static double GetSkillshotWidth(Skillshot skillshot)
    {
        return skillshot.Match(
            linear: l => l.Width,
            circular: c => c.Radius * 2,
            cone: _ => 0,
            arc: a => a.Width,
            rectangle: r => r.Width,
            vectorRectangle: v => v.Width);
    }

    private static string CreateCacheKey(Skillshot skillshot, Point2D caster, MovementState target)
    {
        var targetPos = target.GetPosition();
        var targetVel = target.GetVelocity();

        // Round positions to reduce cache misses from tiny movements
        var cx = Math.Round(caster.X / 10) * 10;
        var cy = Math.Round(caster.Y / 10) * 10;
        var tx = Math.Round(targetPos.X / 10) * 10;
        var ty = Math.Round(targetPos.Y / 10) * 10;
        var vx = Math.Round(targetVel.X / 50) * 50;
        var vy = Math.Round(targetVel.Y / 50) * 50;

        return $"{skillshot.GetHashCode()}:{cx},{cy}:{tx},{ty}:{vx},{vy}";
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
}
