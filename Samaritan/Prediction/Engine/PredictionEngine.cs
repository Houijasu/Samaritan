namespace Samaritan.Prediction.Engine;

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
    // How deep inside the hitbox rim the planned closest approach sits, as the
    // kept fraction of the effective radius (1.0 would be an exact tangent)
    private const double RearGrazeMargin = 0.95; // robust default - ~5% penetration

    // NearestRear: target a closest approach of R*(1 - epsilon) in the sim frame
    // (~0.25 units for R = 100), accepting up to this many extra units of depth
    // from the bisection tolerance
    private const double TangentGrazeEpsilon = 0.0025;
    private const double TangentGapToleranceUnits = 0.1;

    // Minima: how much earlier than the Gagong reference contact Minima's own
    // contact may land, in seconds - the CAP on the contact win. The actual
    // margin is a small share of Gagong's own slack above the global contact
    // floor, so where Gagong is near-optimal Minima ties it instead of paying
    // depth for a sub-millisecond win (the contact-vs-depth frontier is steep
    // near the floor)
    private const double MinimaGagongContactMargin = 0.002;
    private const double MinimaGagongSlackShare = 0.15;

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
        double hitboxRadius,
        ProjectileAimMode aimMode = ProjectileAimMode.RearGraze)
    {
        // Check cache (key construction skipped entirely when caching is off)
        string? cacheKey = null;
        if (_cache is not null)
        {
            cacheKey = $"{CreateCacheKey(skillshot, casterPosition, targetState, hitboxRadius)}:{(int)aimMode}";
            if (_cache.TryGet(cacheKey, out var cachedResult))
            {
                return cachedResult;
            }
        }

        // Get skillshot parameters
        var range = skillshot.GetMaxRange();
        var projectileSpeed = skillshot.GetProjectileSpeed();

        // Add network compensation: ping + tick uncertainty + reaction buffer
        // This accounts for the time between seeing the target and the server processing your cast
        var effectiveDelay = skillshot.GetDelay() + _config.NetworkCompensationDelay;

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
        var effectiveRadius = skillshot.GetEffectiveRadius(hitboxRadius);

        PredictionResult result;

        // For waypoint paths with remaining segments, use the specialized solver
        // (handles direction changes). A finished path falls through to the
        // standard solvers, which treat the target as stationary at the last waypoint.
        if (targetState is MovementState.Pathing pathing && pathing.GetPathSegments().Any())
        {
            result = SolveWaypointInterception(
                skillshot, casterPosition, pathing, effectiveRadius, effectiveDelay, range);
        }
        // For traveling projectiles with moving targets, solve interception directly
        else if (targetSpeed > 1.0 && projectileSpeed is double movingSolverSpeed)
        {
            // Placed effects detonate where aimed, so they intercept the target center.
            // Projectiles aim BEHIND the target: rear-edge graze (RearGraze), most
            // tangent graze (NearestRear), or earliest rear-side contact (Optimal).
            var movingResult = IsPlacedEffect(skillshot)
                ? SolveCenteredInterception(
                    casterPosition, targetPosition, targetVelocity, effectiveRadius,
                    movingSolverSpeed, effectiveDelay)
                : aimMode switch
                {
                    ProjectileAimMode.NearestRear =>
                        SolveTangentRearInterception(
                            casterPosition, targetPosition, targetVelocity, effectiveRadius,
                            movingSolverSpeed, effectiveDelay, _config.NetworkCompensationDelay, range)
                        ?? SolveTrailingEdgeInterception(
                            casterPosition, targetPosition, targetVelocity, effectiveRadius,
                            movingSolverSpeed, effectiveDelay, RearGrazeMargin, 0),
                    ProjectileAimMode.Optimal =>
                        SolveOptimalRearInterception(
                            casterPosition, targetPosition, targetVelocity, effectiveRadius,
                            movingSolverSpeed, effectiveDelay, _config.NetworkCompensationDelay, range,
                            hitboxRadius)
                        ?? SolveTrailingEdgeInterception(
                            casterPosition, targetPosition, targetVelocity, effectiveRadius,
                            movingSolverSpeed, effectiveDelay, RearGrazeMargin, 0),
                    ProjectileAimMode.Minima =>
                        SolveMinimaInterception(
                            casterPosition, targetPosition, targetVelocity, effectiveRadius,
                            movingSolverSpeed, effectiveDelay, _config.NetworkCompensationDelay, range,
                            GagongCastPosition(skillshot, casterPosition, targetState, hitboxRadius))
                        ?? SolveTrailingEdgeInterception(
                            casterPosition, targetPosition, targetVelocity, effectiveRadius,
                            movingSolverSpeed, effectiveDelay, RearGrazeMargin, 0),
                    _ => SolveExactTimeRearInterception(
                            casterPosition, targetPosition, targetVelocity, effectiveRadius,
                            movingSolverSpeed, effectiveDelay, _config.NetworkCompensationDelay)
                        ?? SolveTrailingEdgeInterception(
                            casterPosition, targetPosition, targetVelocity, effectiveRadius,
                            movingSolverSpeed, effectiveDelay, RearGrazeMargin, 0)
                };

            if (movingResult.HasValue)
            {
                var (interceptionTime, castPosition) = movingResult.Value;
                var aimDistance = casterPosition.DistanceTo(castPosition);

                if (aimDistance > range)
                {
                    result = new PredictionResult.OutOfRange(aimDistance, range);
                }
                else
                {
                    var predictedPosition = targetState.PredictPosition(interceptionTime);

                    result = new PredictionResult.Hit(
                        interceptionTime,
                        castPosition,
                        predictedPosition,
                        ComputeConfidence(casterPosition, castPosition, targetSpeed, movingSolverSpeed));
                }
            }
            else
            {
                result = new PredictionResult.Unreachable("No valid interception found for moving target");
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
                    // Placed effects center on the target; projectiles aim at the near edge
                    var castPosition = IsPlacedEffect(skillshot)
                        ? predictedPosition
                        : ComputeStaticAimPoint(casterPosition, predictedPosition, effectiveRadius);

                    result = new PredictionResult.Hit(
                        adjustedTime,
                        castPosition,
                        predictedPosition,
                        solution.Value.Confidence);
                }
            }
        }

        // Cache result
        if (cacheKey is not null)
        {
            _cache!.Set(cacheKey, result);
        }

        return result;
    }

    /// <summary>
    /// Solves interception for targets following waypoint paths by delegating to
    /// the shared segment-by-segment algorithm in <see cref="WaypointInterceptionSolver"/>.
    /// </summary>
    private static PredictionResult SolveWaypointInterception(
        Skillshot skillshot,
        Point2D casterPosition,
        MovementState.Pathing pathing,
        double effectiveRadius,
        double effectiveDelay,
        double range)
    {
        var segments = pathing.GetPathSegments().ToList();

        if (segments.Count == 0)
            return new PredictionResult.Unreachable("No path segments");

        // Instant skillshots (cones, zero-speed AoE) apply at the end of the delay -
        // aim at the position the target reaches by then
        if (skillshot.GetProjectileSpeed() is not double projectileSpeed)
        {
            var instantPosition = pathing.PredictPosition(effectiveDelay);
            var instantDistance = casterPosition.DistanceTo(instantPosition);

            if (instantDistance > range)
                return new PredictionResult.OutOfRange(instantDistance, range);

            return new PredictionResult.Hit(
                effectiveDelay,
                instantPosition,
                instantPosition,
                ComputeConfidence(casterPosition, instantPosition, pathing.Speed, double.PositiveInfinity));
        }

        var solution = WaypointInterceptionSolver.SolveOnPath(
            casterPosition, segments, effectiveDelay, projectileSpeed, range, effectiveRadius);

        if (solution is null)
            return new PredictionResult.Unreachable("No valid interception found on path");

        var (totalTime, pathAimPoint) = solution.Value;
        var predictedPosition = pathing.PredictPosition(totalTime);

        // Placed area effects detonate where aimed - center them on the predicted
        // position for maximum margin; projectiles keep the trailing-edge aim point
        var aimPoint = IsPlacedEffect(skillshot) && casterPosition.DistanceTo(predictedPosition) <= range
            ? predictedPosition
            : pathAimPoint;

        return new PredictionResult.Hit(
            totalTime,
            aimPoint,
            predictedPosition,
            ComputeConfidence(casterPosition, aimPoint, pathing.Speed, projectileSpeed));
    }

    /// <summary>
    /// Placed effects land at the aim position itself (rather than sweeping a line
    /// from the caster), so they should be centered on the target.
    /// </summary>
    private static bool IsPlacedEffect(Skillshot skillshot) =>
        skillshot is Skillshot.Circular or Skillshot.Rectangle or Skillshot.VectorRectangle;

    /// <summary>
    /// Solves interception of the target center for a target moving in a straight
    /// line. Finds the earliest time T >= castDelay at which a projectile launched
    /// after castDelay comes within effectiveRadius of the target center:
    ///   |D + V*T| = speed * (T - delay) + R
    /// which expands to aT² + bT + c = 0 with k = speed*delay - R:
    ///   a = |V|² - speed², b = 2(D·V + speed*k), c = |D|² - k²
    /// Used for placed effects, which detonate where they are aimed.
    /// </summary>
    private static (double Time, Point2D AimPoint)? SolveCenteredInterception(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double effectiveRadius,
        double projectileSpeed,
        double castDelay)
    {
        // Target already inside the effective radius when the projectile launches
        var positionAtLaunch = targetPosition + targetVelocity.ScaleBy(castDelay);
        if (casterPosition.DistanceTo(positionAtLaunch) <= effectiveRadius)
            return (castDelay, positionAtLaunch);

        var displacement = targetPosition - casterPosition;
        var launchOffset = projectileSpeed * castDelay - effectiveRadius;

        var quadA = targetVelocity.DotProduct(targetVelocity) - projectileSpeed * projectileSpeed;
        var quadB = 2.0 * (displacement.DotProduct(targetVelocity) + projectileSpeed * launchOffset);
        var quadC = displacement.DotProduct(displacement) - launchOffset * launchOffset;

        var interceptTime = MinRootAtOrAfter(quadA, quadB, quadC, castDelay);
        if (interceptTime is not double time)
            return null;

        var aimPoint = targetPosition + targetVelocity.ScaleBy(time);

        return (time, aimPoint);
    }

    /// <summary>
    /// Solves interception aiming at the rear edge of the target's hitbox.
    /// The aim point trails the predicted center along the movement direction by a
    /// lead L chosen so the missile front grazes the REAR of the hitbox instead of
    /// clipping the caster-side flank on the way in. The graze condition follows
    /// from the relative motion of front and center near arrival:
    ///   gap²(τ) = relSq·τ² - 2L(v - p·cosφ)τ + L²,  relSq = p² + v² - 2pv·cosφ
    /// (τ = time before arrival, φ = angle between path and missile ray), whose
    /// minimum equals R when L = R·√relSq / (p·sinφ). The grazeMargin sets how
    /// deep inside the edge the closest approach is planned, and leadCompensation
    /// lengthens the lead to cancel the launch skew between the prediction frame
    /// and the actual cast. The reported time is the FIRST contact (largest τ
    /// with gap = R), so predicted and actual hit times agree.
    /// </summary>
    private static (double Time, Point2D AimPoint)? SolveTrailingEdgeInterception(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double effectiveRadius,
        double projectileSpeed,
        double castDelay,
        double grazeMargin,
        double leadCompensation)
    {
        const int LeadRefinements = 5;             // lead depends on the ray, which depends on the lead
        const double MaxLeadFactor = 2.0;          // cap rear-aim on near-head-on approaches
        const double LeadConvergenceTolerance = 0.05; // must stay well below the planned graze margin

        // Target's hitbox already covers the launch point - immediate hit
        var centerAtLaunch = targetPosition + targetVelocity.ScaleBy(castDelay);
        if (casterPosition.DistanceTo(centerAtLaunch) <= effectiveRadius)
            return (castDelay, centerAtLaunch);

        var targetSpeed = targetVelocity.Length;
        var pathDirection = targetVelocity.Normalize();
        var speedSq = projectileSpeed * projectileSpeed;
        var launchDistance = projectileSpeed * castDelay;

        var lead = effectiveRadius + leadCompensation;
        double arrivalTime = 0;
        Point2D aimPoint = default;

        for (var iteration = 0; iteration <= LeadRefinements; iteration++)
        {
            // Missile arrival at the lead point: |D - L*v̂ + V*T| = p*(T - d)
            //   a = |V|² - p², b = 2(D'·V + p²·d), c = |D'|² - (p·d)²
            var displacement = targetPosition - casterPosition - pathDirection.ScaleBy(lead);
            var quadA = targetVelocity.DotProduct(targetVelocity) - speedSq;
            var quadB = 2.0 * (displacement.DotProduct(targetVelocity) + projectileSpeed * launchDistance);
            var quadC = displacement.DotProduct(displacement) - launchDistance * launchDistance;

            if (MinRootAtOrAfter(quadA, quadB, quadC, castDelay) is not double time)
                return null;

            arrivalTime = time;
            aimPoint = targetPosition - pathDirection.ScaleBy(lead) + targetVelocity.ScaleBy(time);

            if (iteration == LeadRefinements)
                break;

            var newLead = RearGrazeLead(
                casterPosition, aimPoint, pathDirection, targetSpeed, projectileSpeed, effectiveRadius);
            newLead = Math.Min(newLead * grazeMargin, MaxLeadFactor * effectiveRadius) + leadCompensation;

            if (Math.Abs(newLead - lead) < LeadConvergenceTolerance)
                break;

            lead = newLead;
        }

        // First contact: largest τ (earliest time) with gap(τ) = R
        var contactOffset = FirstContactOffset(
            casterPosition, aimPoint, pathDirection, targetSpeed, projectileSpeed, effectiveRadius, lead);
        var contactTime = Math.Max(castDelay, arrivalTime - contactOffset);

        return (contactTime, aimPoint);
    }

    /// <summary>
    /// The lead for which the missile front tangentially grazes the rear edge of
    /// the hitbox: L = R·√(p² + v² - 2pv·cosφ) / (p·sinφ). Falls back to the
    /// effective radius for chase or near-parallel geometries, where the front
    /// overtakes the hitbox from directly behind and the base lead already
    /// contacts the rear edge at arrival.
    /// </summary>
    private static double RearGrazeLead(
        Point2D casterPosition,
        Point2D aimPoint,
        Vector2D pathDirection,
        double targetSpeed,
        double projectileSpeed,
        double effectiveRadius)
    {
        var toAim = aimPoint - casterPosition;
        if (toAim.Length < 1e-6)
            return effectiveRadius;

        var rayDirection = toAim.Normalize();
        var cosPhi = pathDirection.DotProduct(rayDirection);
        var sinPhiSq = Math.Max(0, 1 - cosPhi * cosPhi);

        // Chase geometry (closest approach is at/after arrival) or near-parallel ray
        if (targetSpeed - projectileSpeed * cosPhi <= 0 || sinPhiSq < 0.0025)
            return effectiveRadius;

        var relativeSpeedSq = projectileSpeed * projectileSpeed + targetSpeed * targetSpeed
                            - 2 * projectileSpeed * targetSpeed * cosPhi;

        return effectiveRadius * Math.Sqrt(relativeSpeedSq) / (projectileSpeed * Math.Sqrt(sinPhiSq));
    }

    /// <summary>
    /// Time between first contact (front-to-center gap reaching the effective
    /// radius) and arrival at the aim point: the largest root of
    /// relSq·τ² - 2L(v - p·cosφ)τ + (L² - R²) = 0. Zero when contact happens at
    /// arrival itself (chase geometries).
    /// </summary>
    private static double FirstContactOffset(
        Point2D casterPosition,
        Point2D aimPoint,
        Vector2D pathDirection,
        double targetSpeed,
        double projectileSpeed,
        double effectiveRadius,
        double lead)
    {
        var toAim = aimPoint - casterPosition;
        if (toAim.Length < 1e-6)
            return 0;

        var rayDirection = toAim.Normalize();
        var cosPhi = pathDirection.DotProduct(rayDirection);
        var relativeSpeedSq = projectileSpeed * projectileSpeed + targetSpeed * targetSpeed
                            - 2 * projectileSpeed * targetSpeed * cosPhi;

        if (relativeSpeedSq < 1e-9)
            return 0;

        var halfB = lead * (targetSpeed - projectileSpeed * cosPhi);
        var discriminant = halfB * halfB - relativeSpeedSq * (lead * lead - effectiveRadius * effectiveRadius);

        if (discriminant < 0)
            return 0;

        return Math.Max(0, (halfB + Math.Sqrt(discriminant)) / relativeSpeedSq);
    }

    /// <summary>
    /// Solves the most tangent rear graze by searching over the missile RAY ANGLE
    /// directly (for a linear skillshot only the direction matters). In the sim
    /// frame (raw delay d = effectiveDelay - netComp) the center-minus-front
    /// offset is affine in flight time s: g(s) = G0 + W·s with
    /// G0 = (P - C) + V·d and W = V - p·r̂(θ), so the closest approach over the
    /// range-clamped flight is a closed form. Bisection finds the ray whose
    /// closest approach equals R·(1 - epsilon), rotating from a guaranteed-deep
    /// inner ray (through the classical R = 0 interception point, minGap = 0)
    /// toward the REAR side of the hit arc. Returns null when no in-range
    /// tangent ray exists (caller falls back to the rear-graze solve).
    /// </summary>
    private static (double Time, Point2D AimPoint)? SolveTangentRearInterception(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double effectiveRadius,
        double projectileSpeed,
        double effectiveDelay,
        double networkCompensation,
        double range)
    {
        var rawDelay = Math.Max(0, effectiveDelay - networkCompensation);
        var launchOffset = (targetPosition - casterPosition) + targetVelocity.ScaleBy(rawDelay);
        var maxFlight = range / projectileSpeed;

        // Target's hitbox already covers the launch point - immediate hit
        if (launchOffset.Length <= effectiveRadius)
            return (effectiveDelay, targetPosition + targetVelocity.ScaleBy(effectiveDelay));

        var targetGap = effectiveRadius * (1 - TangentGrazeEpsilon);

        double GapAt(Vector2D ray) => ClampedRayMinGap(
            launchOffset, targetVelocity - ray.ScaleBy(projectileSpeed), maxFlight);

        if (FindDeepInnerRay(
                launchOffset, targetPosition - casterPosition, targetVelocity,
                projectileSpeed, rawDelay, targetGap, GapAt) is not Vector2D innerRayValue)
        {
            return null;
        }

        var innerRay = (Vector2D?)innerRayValue;

        // Rotating in this direction exits the hit arc through its REAR boundary
        var pathDirection = targetVelocity.Normalize();
        var cross = innerRay.Value.X * pathDirection.Y - innerRay.Value.Y * pathDirection.X;
        var rotationSign = Math.Abs(cross) < 1e-9 ? 1.0 : -Math.Sign(cross);

        for (var attempt = 0; attempt < 2; attempt++, rotationSign = -rotationSign)
        {
            if (BisectTangentRay(innerRay.Value, rotationSign, targetGap, GapAt) is not Vector2D tangentRay)
                continue;

            // First contact in the sim frame defines the cast position (rear rim)
            var relativeVelocity = targetVelocity - tangentRay.ScaleBy(projectileSpeed);
            var contactFlight = MinRootAtOrAfter(
                relativeVelocity.DotProduct(relativeVelocity),
                2.0 * launchOffset.DotProduct(relativeVelocity),
                launchOffset.DotProduct(launchOffset) - effectiveRadius * effectiveRadius,
                0);
            if (contactFlight is not double flight)
                continue;

            // Safety: the contact must not sit on the leading side of the hitbox.
            // Broadside (~0) is legal for head-on and chase geometries.
            var contactPoint = casterPosition + tangentRay.ScaleBy(projectileSpeed * flight);
            var centerAtContact = casterPosition + launchOffset + targetVelocity.ScaleBy(flight);
            var rearDot = (contactPoint - centerAtContact).DotProduct(targetVelocity);
            if (rearDot > 0.3 * effectiveRadius * targetVelocity.Length)
                continue;

            var castPosition = casterPosition + tangentRay.ScaleBy(Math.Max(1.0, projectileSpeed * flight));
            var time = PredictionFrameContactTime(
                casterPosition, targetPosition, targetVelocity, effectiveRadius,
                projectileSpeed, effectiveDelay, tangentRay, maxFlight);

            return (time, castPosition);
        }

        return null;
    }

    /// <summary>
    /// Solves the default (rear-at-exact-time) aim: the missile's first contact
    /// happens at the EXACT method's minimal interception time, with the contact
    /// point swung toward the rear rim of the hitbox. Contact time as a function
    /// of ray angle is flat around its minimum, so the rear swing is free: the
    /// launch cushion (the prediction budgets network compensation that the
    /// actual cast does not wait for) advances the front by p·netComp units,
    /// which rotates the touch point around the rim at zero time cost.
    /// Construction: solve the centered interception for T_exact, then intersect
    /// the front-travel circle (radius p·(T_exact - rawDelay) around the caster)
    /// with the hitbox rim (radius R around the center at T_exact) and take the
    /// rear-side point; verify the touch is the FIRST contact on that ray.
    /// Returns null when the construction degenerates (caller falls back to the
    /// rear-graze tangency).
    /// </summary>
    private static (double Time, Point2D AimPoint)? SolveExactTimeRearInterception(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double effectiveRadius,
        double projectileSpeed,
        double effectiveDelay,
        double networkCompensation)
    {
        var exact = SolveCenteredInterception(
            casterPosition, targetPosition, targetVelocity, effectiveRadius, projectileSpeed, effectiveDelay);
        if (exact is null)
            return null;

        var exactTime = exact.Value.Time;
        var center = targetPosition + targetVelocity.ScaleBy(exactTime);
        var toCenter = center - casterPosition;
        var centerDistance = toCenter.Length;

        // Caster effectively inside the hitbox - aim at the body
        if (centerDistance <= effectiveRadius || centerDistance < 1e-9)
            return (exactTime, center);

        // Front travel by the exact moment in the actual-cast frame (raw delay):
        // the cushion pushes it past the solver-frame near edge, which is what
        // swings the rim touch toward the rear
        var rawDelay = Math.Max(0, effectiveDelay - networkCompensation);
        var frontDistance = Math.Clamp(
            projectileSpeed * (exactTime - rawDelay),
            centerDistance - effectiveRadius,
            centerDistance + effectiveRadius);

        // Circle-circle intersection: |X - caster| = frontDistance, |X - center| = R
        var cosAtCaster = (frontDistance * frontDistance + centerDistance * centerDistance
                         - effectiveRadius * effectiveRadius) / (2 * frontDistance * centerDistance);
        cosAtCaster = Math.Clamp(cosAtCaster, -1.0, 1.0);

        var baseAngle = Math.Atan2(toCenter.Y, toCenter.X);
        var halfAngle = Math.Acos(cosAtCaster);

        var candidate1 = casterPosition + new Vector2D(
            Math.Cos(baseAngle + halfAngle), Math.Sin(baseAngle + halfAngle)).ScaleBy(frontDistance);
        var candidate2 = casterPosition + new Vector2D(
            Math.Cos(baseAngle - halfAngle), Math.Sin(baseAngle - halfAngle)).ScaleBy(frontDistance);

        // Rear rim: the candidate behind the center relative to the movement direction
        var aimPoint = (candidate1 - center).DotProduct(targetVelocity)
                     <= (candidate2 - center).DotProduct(targetVelocity)
            ? candidate1
            : candidate2;

        // The rim touch must be the FIRST contact on this ray (entry crossing),
        // otherwise the actual hit would land earlier than the exact time
        var toAim = aimPoint - casterPosition;
        if (toAim.Length < 1e-9)
            return null;

        var ray = toAim.Normalize();
        var relativeVelocity = targetVelocity - ray.ScaleBy(projectileSpeed);
        var launchOffset = (targetPosition - casterPosition) + targetVelocity.ScaleBy(rawDelay);
        var firstContact = MinRootAtOrAfter(
            relativeVelocity.DotProduct(relativeVelocity),
            2.0 * launchOffset.DotProduct(relativeVelocity),
            launchOffset.DotProduct(launchOffset) - effectiveRadius * effectiveRadius,
            0);

        var touchFlight = exactTime - rawDelay;
        if (firstContact is not double contactFlight || contactFlight < touchFlight - 1e-3)
            return null;

        return (exactTime, aimPoint);
    }

    /// <summary>
    /// Solves the earliest rear-side contact: among all rays whose pass lands on
    /// the rear half of the hitbox AND penetrates no deeper than the target's
    /// bounding radius (the "HIT by" depth stays below hitboxRadius), finds the
    /// one with the smallest first-contact time (sim frame), and casts at the
    /// contact point itself - the closest cast position to the target. Sweeps
    /// the hit arc (bounded by the two near-tangent rays) coarsely, then refines
    /// around the best sample. Returns null when no valid ray exists.
    /// </summary>
    private static (double Time, Point2D AimPoint)? SolveOptimalRearInterception(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double effectiveRadius,
        double projectileSpeed,
        double effectiveDelay,
        double networkCompensation,
        double range,
        double hitboxRadius)
    {
        var rawDelay = Math.Max(0, effectiveDelay - networkCompensation);
        var launchOffset = (targetPosition - casterPosition) + targetVelocity.ScaleBy(rawDelay);
        var maxFlight = range / projectileSpeed;

        // Target's hitbox already covers the launch point - immediate hit
        if (launchOffset.Length <= effectiveRadius)
            return (effectiveDelay, targetPosition + targetVelocity.ScaleBy(effectiveDelay));

        var targetGap = effectiveRadius * (1 - TangentGrazeEpsilon);

        double GapAt(Vector2D ray) => ClampedRayMinGap(
            launchOffset, targetVelocity - ray.ScaleBy(projectileSpeed), maxFlight);

        if (FindDeepInnerRay(
                launchOffset, targetPosition - casterPosition, targetVelocity,
                projectileSpeed, rawDelay, targetGap, GapAt) is not Vector2D innerRay)
        {
            return null;
        }

        // Hit-arc endpoints: the near-tangent rays on both rotation sides
        if (BisectTangentRay(innerRay, 1.0, targetGap, GapAt) is not Vector2D positiveEnd ||
            BisectTangentRay(innerRay, -1.0, targetGap, GapAt) is not Vector2D negativeEnd)
        {
            return null;
        }

        var positiveAngle = SignedAngle(innerRay, positiveEnd);
        var negativeAngle = SignedAngle(innerRay, negativeEnd);

        double? FlightAt(Vector2D ray)
        {
            var relativeVelocity = targetVelocity - ray.ScaleBy(projectileSpeed);
            var flight = MinRootAtOrAfter(
                relativeVelocity.DotProduct(relativeVelocity),
                2.0 * launchOffset.DotProduct(relativeVelocity),
                launchOffset.DotProduct(launchOffset) - effectiveRadius * effectiveRadius,
                0);

            return flight is double f && f <= maxFlight ? f : null;
        }

        // Rear half of the hitbox, with a broadside tolerance. Evaluated at the
        // deepest point of the pass (closest approach), not the first touch: a
        // head-on target is always touched on its leading face first, and even
        // its closest approach sits a few units on the caster side - 5% of R
        // accepts those while still rejecting genuinely frontal passes (~100%).
        var rearTolerance = 0.05 * effectiveRadius * targetVelocity.Length;

        bool PassIsRear(Vector2D ray)
        {
            var relativeVelocity = targetVelocity - ray.ScaleBy(projectileSpeed);
            var relativeSpeedSq = relativeVelocity.DotProduct(relativeVelocity);
            if (relativeSpeedSq < 1e-12)
                return false;

            var closestApproach = Math.Clamp(
                -launchOffset.DotProduct(relativeVelocity) / relativeSpeedSq, 0, maxFlight);
            var centerMinusFront = launchOffset + relativeVelocity.ScaleBy(closestApproach);

            return -centerMinusFront.DotProduct(targetVelocity) <= rearTolerance;
        }

        Vector2D? bestRay = null;
        var bestFlight = double.MaxValue;
        var bestAngle = 0.0;

        // Penetration cap: the deepest point of the pass must stay within the
        // target's bounding radius, so the HIT-by depth never exceeds it
        var minAllowedGap = Math.Max(0, effectiveRadius - hitboxRadius);

        void Sweep(double fromAngle, double toAngle, int steps)
        {
            for (var i = 0; i <= steps; i++)
            {
                var angle = fromAngle + (toAngle - fromAngle) * i / steps;
                var ray = Rotate(innerRay, angle);

                if (FlightAt(ray) is not double flight || flight >= bestFlight)
                    continue;
                if (GapAt(ray) < minAllowedGap)
                    continue;
                if (!PassIsRear(ray))
                    continue;

                bestFlight = flight;
                bestRay = ray;
                bestAngle = angle;
            }
        }

        Sweep(negativeAngle, positiveAngle, 64);
        if (bestRay is null)
            return null;

        var refineStep = (positiveAngle - negativeAngle) / 64;
        Sweep(bestAngle - refineStep, bestAngle + refineStep, 64);

        var finalRay = bestRay.Value;
        var castPosition = casterPosition + finalRay.ScaleBy(Math.Max(1.0, projectileSpeed * bestFlight));
        var time = PredictionFrameContactTime(
            casterPosition, targetPosition, targetVelocity, effectiveRadius,
            projectileSpeed, effectiveDelay, finalRay, maxFlight);

        return (time, castPosition);
    }

    /// <summary>
    /// Solves the Minima aim: the SHALLOWEST pass (minimal HIT BY) among rays
    /// whose first contact beats the Gagong reference contact by
    /// <see cref="MinimaGagongContactMargin"/> - so ACTUAL HIT lands earlier
    /// than Gagong wherever Gagong is above the global contact floor, and the
    /// depth sacrificed for the shallowness is the little the geometry forces.
    /// Where Gagong already sits on the floor (chasing/fleeing geometries,
    /// where its centered interception is optimal), the budget clamps to the
    /// floor and Minima ties Gagong's contact. The gap grows toward the two
    /// tangent endpoints of the hit arc as the contact time grows away from
    /// the arc's deep center, so the optimum sits exactly on the budget
    /// boundary - found by per-side bisection. Without a Gagong reference
    /// (it missed or returned a non-hit), the budget collapses to the plain
    /// earliest contact. Casts at the contact point itself. Returns null when
    /// no in-range hitting ray exists (caller falls back to the rear-graze
    /// solve).
    /// </summary>
    private static (double Time, Point2D AimPoint)? SolveMinimaInterception(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double effectiveRadius,
        double projectileSpeed,
        double effectiveDelay,
        double networkCompensation,
        double range,
        Point2D? gagongCastPosition)
    {
        var rawDelay = Math.Max(0, effectiveDelay - networkCompensation);
        var launchOffset = (targetPosition - casterPosition) + targetVelocity.ScaleBy(rawDelay);
        var maxFlight = range / projectileSpeed;

        // Target's hitbox already covers the launch point - immediate hit
        if (launchOffset.Length <= effectiveRadius)
            return (effectiveDelay, targetPosition + targetVelocity.ScaleBy(effectiveDelay));

        var targetGap = effectiveRadius * (1 - TangentGrazeEpsilon);

        double GapAt(Vector2D ray) => ClampedRayMinGap(
            launchOffset, targetVelocity - ray.ScaleBy(projectileSpeed), maxFlight);

        if (FindDeepInnerRay(
                launchOffset, targetPosition - casterPosition, targetVelocity,
                projectileSpeed, rawDelay, targetGap, GapAt) is not Vector2D innerRay)
        {
            return null;
        }

        // Hit-arc endpoints: the near-tangent rays on both rotation sides
        if (BisectTangentRay(innerRay, 1.0, targetGap, GapAt) is not Vector2D positiveEnd ||
            BisectTangentRay(innerRay, -1.0, targetGap, GapAt) is not Vector2D negativeEnd)
        {
            return null;
        }

        var positiveAngle = SignedAngle(innerRay, positiveEnd);
        var negativeAngle = SignedAngle(innerRay, negativeEnd);

        double? FlightAt(Vector2D ray)
        {
            var relativeVelocity = targetVelocity - ray.ScaleBy(projectileSpeed);
            var flight = MinRootAtOrAfter(
                relativeVelocity.DotProduct(relativeVelocity),
                2.0 * launchOffset.DotProduct(relativeVelocity),
                launchOffset.DotProduct(launchOffset) - effectiveRadius * effectiveRadius,
                0);

            return flight is double f && f <= maxFlight ? f : null;
        }

        // Pass 1: the earliest first contact over the whole hit arc - the
        // answer when the cap is free (narrow arcs), and the fallback when no
        // in-range ray satisfies it. Refined locally around the coarse argmin.
        Vector2D? earlyRay = null;
        var minFlight = double.MaxValue;
        var argminAngle = 0.0;

        void SweepEarliest(double fromAngle, double toAngle, int steps)
        {
            for (var i = 0; i <= steps; i++)
            {
                var angle = fromAngle + (toAngle - fromAngle) * i / steps;
                var ray = Rotate(innerRay, angle);
                if (FlightAt(ray) is double flight && flight < minFlight)
                {
                    minFlight = flight;
                    earlyRay = ray;
                    argminAngle = angle;
                }
            }
        }

        SweepEarliest(negativeAngle, positiveAngle, 64);
        if (earlyRay is null)
            return null;

        var coarseStep = (positiveAngle - negativeAngle) / 64;
        SweepEarliest(argminAngle - coarseStep, argminAngle + coarseStep, 64);

        // Pass 2: the budget - the latest first contact Minima may take. It is
        // the Gagong reference contact minus a margin, clamped to the global
        // floor from pass 1. The margin spends only a small share of Gagong's
        // own slack above that floor on the contact win (capped): where
        // Gagong already sits on the floor, the budget ties it; where Gagong
        // is well above, Minima's contact lands earlier by up to the cap.
        double? budget = null;
        if (gagongCastPosition is Point2D gagongAim)
        {
            var toGagongAim = gagongAim - casterPosition;
            if (toGagongAim.Length > 1e-9
                && FlightAt(toGagongAim.Normalize()) is double gagongFlight)
            {
                var slack = Math.Max(0, gagongFlight - minFlight);
                var margin = Math.Min(MinimaGagongContactMargin, MinimaGagongSlackShare * slack);
                budget = Math.Max(gagongFlight - margin, minFlight);
            }
        }

        // The shallowest pass among rays whose contact fits the budget: gap
        // grows toward the tangent endpoints while flight grows away from the
        // argmin, so the optimum sits exactly ON the budget boundary on each
        // side - bisected below. A coarse arc sweep plus the argmin ray guard
        // against non-monotone geometries. Without a Gagong reference (it
        // missed), the budget collapses to the earliest contact itself.
        var threshold = budget ?? minFlight;

        Vector2D? bestRay = null;
        var bestGap = double.MinValue;
        var bestFlight = 0.0;

        void Consider(Vector2D ray)
        {
            if (FlightAt(ray) is not double flight || flight > threshold)
                return;

            var gap = GapAt(ray);
            if (gap > bestGap)
            {
                bestGap = gap;
                bestFlight = flight;
                bestRay = ray;
            }
        }

        Consider(earlyRay.Value);
        for (var i = 0; i <= 64; i++)
            Consider(Rotate(innerRay, negativeAngle + (positiveAngle - negativeAngle) * i / 64));

        if (budget is double bounded)
        {
            // Boundary bisection per side: the flight grows from the argmin
            // toward the tangent endpoint, so the budget boundary is crossed
            // once per side; lo stays on the within-budget side
            for (var side = 0; side < 2; side++)
            {
                var endAngle = side == 0 ? positiveAngle : negativeAngle;

                // Feasible all the way to the tangent endpoint - it IS the boundary
                if (FlightAt(Rotate(innerRay, endAngle)) is double endFlight && endFlight <= bounded)
                {
                    Consider(Rotate(innerRay, endAngle));
                    continue;
                }

                var lo = argminAngle;
                var hi = endAngle;
                for (var i = 0; i < 50; i++)
                {
                    var mid = (lo + hi) / 2;
                    if (FlightAt(Rotate(innerRay, mid)) is double midFlight && midFlight <= bounded)
                        lo = mid;
                    else
                        hi = mid;
                }

                Consider(Rotate(innerRay, lo));
            }
        }

        var finalRay = bestRay ?? earlyRay.Value;
        var finalFlight = FlightAt(finalRay) ?? minFlight;
        var castPosition = casterPosition + finalRay.ScaleBy(Math.Max(1.0, projectileSpeed * finalFlight));
        var time = PredictionFrameContactTime(
            casterPosition, targetPosition, targetVelocity, effectiveRadius,
            projectileSpeed, effectiveDelay, finalRay, maxFlight);

        return (time, castPosition);
    }

    /// <summary>
    /// Cast position the Gagong port would choose for this state, or null when
    /// it does not produce a hit. Minima uses its contact time as the budget
    /// to beat. The engine is constructed per call with the live config so
    /// ping updates stay reflected.
    /// </summary>
    private Point2D? GagongCastPosition(
        Skillshot skillshot,
        Point2D casterPosition,
        MovementState targetState,
        double hitboxRadius) =>
        new GagongPredictionEngine(_config)
            .PredictFromState(skillshot, casterPosition, targetState, hitboxRadius)
            is PredictionResult.Hit hit ? hit.CastPosition : null;

    /// <summary>
    /// Finds a ray with a deep closest approach (below targetGap) to bracket the
    /// hit arc: the ray through the classical (R = 0) interception point passes
    /// through the origin of relative space (minGap = 0); when that solve has no
    /// root (target as fast as or faster than the missile), a coarse 64-ray scan
    /// looks for any sufficiently deep direction. Null when none exists.
    /// </summary>
    private static Vector2D? FindDeepInnerRay(
        Vector2D launchOffset,
        Vector2D displacement,
        Vector2D targetVelocity,
        double projectileSpeed,
        double rawDelay,
        double targetGap,
        Func<Vector2D, double> gapAt)
    {
        var speedSq = projectileSpeed * projectileSpeed;
        var quadA = targetVelocity.DotProduct(targetVelocity) - speedSq;
        var quadB = 2.0 * (displacement.DotProduct(targetVelocity) + speedSq * rawDelay);
        var quadC = displacement.DotProduct(displacement) - speedSq * rawDelay * rawDelay;

        if (MinRootAtOrAfter(quadA, quadB, quadC, rawDelay) is double interceptTime)
        {
            var interceptPoint = displacement + targetVelocity.ScaleBy(interceptTime);
            if (interceptPoint.Length <= 1e-9)
                return null;

            var ray = interceptPoint.Normalize();
            // Even the deepest ray may not reach the wanted depth (range-limited)
            return gapAt(ray) < targetGap ? ray : null;
        }

        // Coarse scan fallback for geometries where the classical solve has no root
        var bestGap = double.MaxValue;
        var bestRay = new Vector2D(1, 0);
        for (var i = 0; i < 64; i++)
        {
            var candidate = Rotate(new Vector2D(1, 0), 2 * Math.PI * i / 64);
            var gap = gapAt(candidate);
            if (gap < bestGap)
            {
                bestGap = gap;
                bestRay = candidate;
            }
        }

        return bestGap < targetGap ? bestRay : null;
    }

    private static double SignedAngle(Vector2D from, Vector2D to) =>
        Math.Atan2(from.X * to.Y - from.Y * to.X, from.DotProduct(to));

    /// <summary>
    /// Bisects the rotation away from a deep-hitting inner ray until the
    /// closest approach equals the target gap (within tolerance), returning the
    /// hitting-side ray. Null when no outer bracket exists in this direction.
    /// </summary>
    private static Vector2D? BisectTangentRay(
        Vector2D innerRay,
        double rotationSign,
        double targetGap,
        Func<Vector2D, double> gapAt)
    {
        var low = 0.0;
        var high = double.NaN;

        for (var delta = 0.05; delta <= Math.PI; delta *= 2)
        {
            if (gapAt(Rotate(innerRay, rotationSign * delta)) >= targetGap)
            {
                high = delta;
                break;
            }

            low = delta;
        }

        if (double.IsNaN(high))
        {
            if (gapAt(Rotate(innerRay, rotationSign * Math.PI)) < targetGap)
                return null;

            high = Math.PI;
        }

        for (var i = 0; i < 100; i++)
        {
            if (targetGap - gapAt(Rotate(innerRay, rotationSign * low)) <= TangentGapToleranceUnits)
                break;

            var mid = (low + high) / 2;
            if (gapAt(Rotate(innerRay, rotationSign * mid)) >= targetGap)
                high = mid;
            else
                low = mid;
        }

        return Rotate(innerRay, rotationSign * low);
    }

    /// <summary>
    /// Reports the contact time in the prediction frame (effectiveDelay), like
    /// every other mode. The tangent ray is tuned to the sim frame, so in the
    /// prediction frame it can be a near-miss - then the closest-approach moment
    /// is reported instead (the two branches are continuous at the boundary).
    /// </summary>
    private static double PredictionFrameContactTime(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double effectiveRadius,
        double projectileSpeed,
        double effectiveDelay,
        Vector2D ray,
        double maxFlight)
    {
        var launchOffset = (targetPosition - casterPosition) + targetVelocity.ScaleBy(effectiveDelay);
        if (launchOffset.Length <= effectiveRadius)
            return effectiveDelay;

        var relativeVelocity = targetVelocity - ray.ScaleBy(projectileSpeed);
        var contactFlight = MinRootAtOrAfter(
            relativeVelocity.DotProduct(relativeVelocity),
            2.0 * launchOffset.DotProduct(relativeVelocity),
            launchOffset.DotProduct(launchOffset) - effectiveRadius * effectiveRadius,
            0);

        if (contactFlight is double flight)
            return effectiveDelay + flight;

        var relativeSpeedSq = relativeVelocity.DotProduct(relativeVelocity);
        var closestApproach = relativeSpeedSq > 1e-12
            ? Math.Clamp(-launchOffset.DotProduct(relativeVelocity) / relativeSpeedSq, 0, maxFlight)
            : 0;

        return effectiveDelay + closestApproach;
    }

    /// <summary>
    /// Closest approach between missile front and target center over the
    /// range-clamped flight: distance from the origin of relative space to the
    /// segment swept by g(s) = launchOffset + relativeVelocity·s, s in [0, maxFlight].
    /// </summary>
    private static double ClampedRayMinGap(
        Vector2D launchOffset, Vector2D relativeVelocity, double maxFlight)
    {
        var relativeSpeedSq = relativeVelocity.DotProduct(relativeVelocity);
        if (relativeSpeedSq < 1e-12)
            return launchOffset.Length;

        var closestApproach = Math.Clamp(
            -launchOffset.DotProduct(relativeVelocity) / relativeSpeedSq, 0, maxFlight);

        return (launchOffset + relativeVelocity.ScaleBy(closestApproach)).Length;
    }

    private static Vector2D Rotate(Vector2D vector, double angle)
    {
        var cos = Math.Cos(angle);
        var sin = Math.Sin(angle);
        return new Vector2D(vector.X * cos - vector.Y * sin, vector.X * sin + vector.Y * cos);
    }

    /// <summary>
    /// Returns the smallest real root of aT² + bT + c = 0 that is >= minTime,
    /// or null when no such root exists.
    /// </summary>
    private static double? MinRootAtOrAfter(double quadA, double quadB, double quadC, double minTime)
    {
        var (root1, root2) = FindRoots.Quadratic(quadC, quadB, quadA);

        const double ImaginaryTolerance = 1e-9;
        var interceptTime = double.MaxValue;

        if (Math.Abs(root1.Imaginary) < ImaginaryTolerance && root1.Real >= minTime)
            interceptTime = Math.Min(interceptTime, root1.Real);
        if (Math.Abs(root2.Imaginary) < ImaginaryTolerance && root2.Real >= minTime)
            interceptTime = Math.Min(interceptTime, root2.Real);

        return interceptTime < double.MaxValue ? interceptTime : null;
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
        var range = skillshot.GetMaxRange();
        var effectiveDelay = skillshot.GetDelay() + _config.NetworkCompensationDelay;

        // Get target info
        var targetPosition = targetState.GetPosition();
        var targetVelocity = targetState.GetVelocity();
        var targetSpeed = targetVelocity.Length;
        var effectiveRadius = skillshot.GetEffectiveRadius(hitboxRadius);

        // Instant skillshots apply at the end of the delay - no interception equation needed
        if (skillshot.GetProjectileSpeed() is not double skillshotSpeed)
        {
            var instantAim = targetPosition + targetVelocity.ScaleBy(effectiveDelay);
            var instantDistance = casterPosition.DistanceTo(instantAim);

            if (instantDistance > range + effectiveRadius)
                return new PredictionResult.OutOfRange(instantDistance, range);

            return new PredictionResult.Hit(
                effectiveDelay,
                instantAim,
                targetState.PredictPosition(effectiveDelay),
                ComputeConfidence(casterPosition, instantAim, targetSpeed, double.PositiveInfinity));
        }

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

        // The interception time must be >= effectiveDelay (projectile must be launched)
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

    private static string CreateCacheKey(Skillshot skillshot, Point2D caster, MovementState target, double hitboxRadius)
    {
        // Round positions to reduce cache misses from tiny movements
        var cx = Math.Round(caster.X / 10) * 10;
        var cy = Math.Round(caster.Y / 10) * 10;

        // Record ToString includes the type name and all parameters, so distinct
        // skillshots can never alias (unlike GetHashCode, which can collide)
        return $"{skillshot}:{cx},{cy}:{DescribeState(target)}:{Math.Round(hitboxRadius)}";
    }

    private static string DescribeState(MovementState state)
    {
        var pos = state.GetPosition();
        var vel = state.GetVelocity();
        var tx = Math.Round(pos.X / 10) * 10;
        var ty = Math.Round(pos.Y / 10) * 10;
        var vx = Math.Round(vel.X / 50) * 50;
        var vy = Math.Round(vel.Y / 50) * 50;
        var basic = $"{tx},{ty}:{vx},{vy}";

        return state switch
        {
            MovementState.Idle => $"I:{basic}",
            MovementState.Walking => $"W:{basic}",
            MovementState.Dashing d =>
                $"D:{basic}:{Math.Round(d.EndPosition.X)},{Math.Round(d.EndPosition.Y)}:{d.Duration:F2}:{Math.Round(d.Elapsed, 2)}:{d.EaseType}",
            MovementState.Channeling c => $"C:{basic}:{Math.Round(c.Acceleration)}",
            MovementState.Pathing p =>
                $"P:{basic}:{Math.Round(p.Speed)}:{p.CurrentIndex}:{Math.Round(p.ProgressOnSegment, 2)}:{DescribeWaypoints(p.Waypoints)}",
            _ => basic
        };
    }

    private static string DescribeWaypoints(IReadOnlyList<Point2D> waypoints)
    {
        var hash = new HashCode();
        hash.Add(waypoints.Count);
        foreach (var waypoint in waypoints)
        {
            hash.Add(Math.Round(waypoint.X / 10) * 10);
            hash.Add(Math.Round(waypoint.Y / 10) * 10);
        }

        return hash.ToHashCode().ToString();
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
