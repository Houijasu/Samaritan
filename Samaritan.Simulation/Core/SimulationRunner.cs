namespace Samaritan.Simulation.Core;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Collision;
using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;
using Samaritan.Simulation.Metrics;
using Samaritan.Simulation.Scenarios;

/// <summary>
/// Which prediction algorithm the simulation uses.
/// </summary>
public enum PredictionMethod
{
    /// <summary>Current algorithm: rear-edge graze (latest contact).</summary>
    After,

    /// <summary>Current algorithm: most tangent rear graze (minimal HIT-by margin).</summary>
    Nearest,

    /// <summary>Current algorithm: earliest rear-side contact, cast at the contact point.</summary>
    Optimal,

    /// <summary>Port of the community "Gagong" Lua prediction routine, for comparison.</summary>
    Gagong
}

/// <summary>
/// Runs the skillshot prediction simulation.
/// </summary>
public class SimulationRunner
{
    private readonly PredictionEngine _engine = new();
    private readonly GagongPredictionEngine _gagongEngine = new();
    private readonly SimulationLogger _logger = new();
    private Scenario? _scenario;

    public SimulationState State { get; } = new();

    /// <summary>
    /// The prediction algorithm currently in use (Before = legacy, After = current).
    /// </summary>
    public PredictionMethod Method { get; private set; } = PredictionMethod.After;

    /// <summary>
    /// Cycles between the prediction algorithms
    /// (After -> Nearest -> Optimal -> Gagong) and recomputes the prediction
    /// for the loaded scenario.
    /// </summary>
    public void CycleMethod()
    {
        Method = Method switch
        {
            PredictionMethod.After => PredictionMethod.Nearest,
            PredictionMethod.Nearest => PredictionMethod.Optimal,
            PredictionMethod.Optimal => PredictionMethod.Gagong,
            _ => PredictionMethod.After
        };
        Reset();
    }

    /// <summary>
    /// Load a scenario and prepare for simulation.
    /// </summary>
    public void LoadScenario(Scenario scenario)
    {
        _scenario = scenario;
        Reset();
    }

    /// <summary>
    /// Reset the simulation to initial state.
    /// </summary>
    public void Reset()
    {
        if (_scenario is null) return;

        State.Reset();
        State.Phase = SimulationPhase.Ready;

        // Set initial target position
        State.TargetPosition = _scenario.TargetMovement.GetPosition(0);
        State.TargetVelocity = _scenario.TargetMovement.GetVelocity(0);

        // Compute prediction
        ComputePrediction();
    }

    private void ComputePrediction()
    {
        if (_scenario is null) return;

        var velocity = _scenario.TargetMovement.GetVelocity(0);
        MovementState movementState;

        // For waypoint patterns, create Pathing state directly with all waypoints
        if (_scenario.TargetMovement is MovementPattern.Waypoints waypoints)
        {
            movementState = new MovementState.Pathing(
                Waypoints: waypoints.Points,
                Speed: waypoints.Speed,
                CurrentIndex: 1,        // Moving toward second waypoint
                ProgressOnSegment: 0);  // At start of first segment
        }
        else
        {
            // For other patterns, use tracker to infer movement
            var tracker = new MovementTracker { HitboxRadius = _scenario.HitboxRadius };
            tracker.Update(_scenario.TargetMovement.GetPosition(0), 0);

            if (velocity.Length > 0.001)
            {
                var nextPos = _scenario.TargetMovement.GetPosition(0.1);
                tracker.Update(nextPos, 0.1);
            }

            movementState = tracker.CurrentState;

            // Pin the inferred state back to the simulation's t = 0 ground truth:
            // the tracker's latest sample is at t = 0.1, which would put the
            // engine's model 0.1 s (~35 units) ahead of the replayed target
            if (movementState is MovementState.Walking walking)
            {
                movementState = walking with { Position = _scenario.TargetMovement.GetPosition(0) };
            }
        }

        // Get prediction using the movement state and the selected algorithm
        State.Prediction = Method switch
        {
            PredictionMethod.Nearest => _engine.PredictFromState(
                _scenario.Skillshot,
                _scenario.CasterPosition,
                movementState,
                _scenario.HitboxRadius,
                ProjectileAimMode.NearestRear),
            PredictionMethod.Optimal => _engine.PredictFromState(
                _scenario.Skillshot,
                _scenario.CasterPosition,
                movementState,
                _scenario.HitboxRadius,
                ProjectileAimMode.Optimal),
            PredictionMethod.Gagong => _gagongEngine.PredictFromState(
                _scenario.Skillshot,
                _scenario.CasterPosition,
                movementState,
                _scenario.HitboxRadius),
            _ => _engine.PredictFromState(
                _scenario.Skillshot,
                _scenario.CasterPosition,
                movementState,
                _scenario.HitboxRadius)
        };

        State.Prediction.Match(
            hit: h =>
            {
                State.CastPosition = h.CastPosition;
                State.PredictedTargetPosition = h.PredictedPosition;

                // Calculate interception angle geometry
                var targetPos = _scenario.TargetMovement.GetPosition(0);
                var diff = targetPos - _scenario.CasterPosition;
                var targetSpeed = velocity.Length;

                if (diff.Length > 0.001 && targetSpeed > 0.001)
                {
                    State.CosTheta = diff.DotProduct(velocity) / (diff.Length * targetSpeed);
                    State.SinTheta = Math.Sqrt(Math.Max(0, 1.0 - State.CosTheta.Value * State.CosTheta.Value));
                }
            },
            outOfRange: _ => { },
            unreachable: _ => { });

        // Get exact prediction for comparison with fast approximation
        var exactResult = _engine.PredictExact(
            _scenario.Skillshot,
            _scenario.CasterPosition,
            movementState,
            _scenario.HitboxRadius);

        if (exactResult is PredictionResult.Hit exactHit)
        {
            State.ExactPredictedPosition = exactHit.PredictedPosition;
            State.ExactPredictedTime = exactHit.InterceptionTime;
        }

        // Log results for comparison
        _logger.LogComparison(_scenario.Name, State.Prediction, exactResult);

        ComputeGrazeMetrics();
    }

    /// <summary>
    /// Scans the simulated flight (raw-delay launch, matching UpdateFlying) for
    /// the closest approach between the missile front and the target center, so
    /// the HUD can show how far the shot is from the tangency boundary.
    /// </summary>
    private void ComputeGrazeMetrics()
    {
        if (_scenario is null) return;
        if (State.Prediction is not PredictionResult.Hit hit) return;
        if (_scenario.Skillshot is not Skillshot.Linear linear) return;

        var caster = _scenario.CasterPosition;
        var toCast = hit.CastPosition - caster;
        if (toCast.Length < 1e-6) return;

        var ray = toCast.Normalize();
        var effectiveRadius = _scenario.Skillshot.GetEffectiveRadius(_scenario.HitboxRadius);
        var delay = _scenario.Skillshot.GetDelay();
        double speed = linear.Speed;
        double range = linear.Range;

        // Segment-based continuous minimum (same relative-motion math as the
        // simulator's swept collision check), so the readout and the sim agree
        // exactly even for grazes thinner than the scan step
        var origin = new Point2D(0, 0);
        var minGap = double.MaxValue;
        var previousFront = caster;
        var previousCenter = _scenario.TargetMovement.GetPosition(delay);

        for (var t = delay + 0.001; t <= hit.InterceptionTime + 1.0; t += 0.001)
        {
            var travelled = speed * (t - delay);
            if (travelled > range) break;

            var front = caster + ray.ScaleBy(travelled);
            var center = _scenario.TargetMovement.GetPosition(t);

            var startOffset = previousFront - previousCenter;
            var endOffset = front - center;
            var gap = LinearCollisionDetector.PointToSegmentDistance(
                origin,
                new Point2D(startOffset.X, startOffset.Y),
                new Point2D(endOffset.X, endOffset.Y));
            if (gap < minGap) minGap = gap;

            previousFront = front;
            previousCenter = center;
        }

        if (minGap >= double.MaxValue) return;

        var velocity = _scenario.TargetMovement.GetVelocity(0);
        double? approachAngle = null;
        if (velocity.Length > 1)
        {
            var cosPhi = Math.Clamp(velocity.Normalize().DotProduct(ray), -1.0, 1.0);
            approachAngle = Math.Acos(cosPhi) * 180.0 / Math.PI;
        }

        State.GrazeGap = minGap;
        State.GrazeRadius = effectiveRadius;
        State.ApproachAngleDegrees = approachAngle;
    }

    /// <summary>
    /// Update the simulation by the given time step.
    /// </summary>
    public void Update(double deltaTime)
    {
        if (_scenario is null) return;

        switch (State.Phase)
        {
            case SimulationPhase.Ready:
                State.Phase = SimulationPhase.Predicting;
                break;

            case SimulationPhase.Predicting:
                // Immediately start casting
                State.Phase = SimulationPhase.Casting;
                break;

            case SimulationPhase.Casting:
                UpdateCasting(deltaTime);
                break;

            case SimulationPhase.Flying:
                UpdateFlying(deltaTime);
                break;

            case SimulationPhase.Complete:
                break;
        }
    }

    private void UpdateCasting(double deltaTime)
    {
        if (_scenario is null) return;

        State.Time += deltaTime;

        // Update target position
        State.TargetPosition = _scenario.TargetMovement.GetPosition(State.Time);
        State.TargetVelocity = _scenario.TargetMovement.GetVelocity(State.Time);

        // Get skillshot delay
        var delay = _scenario.Skillshot.GetDelay();

        // Check if delay has elapsed
        if (State.Time >= delay)
        {
            State.ProjectileLaunched = true;
            State.ProjectileLaunchTime = delay;
            State.ProjectilePosition = _scenario.CasterPosition;
            State.Phase = SimulationPhase.Flying;
        }
    }

    private void UpdateFlying(double deltaTime)
    {
        if (_scenario is null) return;

        var previousTime = State.Time;
        var previousTarget = State.TargetPosition;

        State.Time += deltaTime;

        // Update target position
        State.TargetPosition = _scenario.TargetMovement.GetPosition(State.Time);
        State.TargetVelocity = _scenario.TargetMovement.GetVelocity(State.Time);

        // Update projectile position
        UpdateProjectile();

        // Check for collision. Linear skillshots use a continuous swept check
        // (both missile and target motion within the frame), so grazing contacts
        // cannot slip between two frame checks; other shapes use the detector.
        if (_scenario.Skillshot is Skillshot.Linear)
        {
            if (CheckLinearContinuousCollision(previousTime, previousTarget, out var contactFraction))
            {
                State.ActualHitTime = previousTime + contactFraction * deltaTime;
                State.ActualHitPosition =
                    previousTarget + (State.TargetPosition - previousTarget).ScaleBy(contactFraction);
                State.Phase = SimulationPhase.Complete;
                return;
            }
        }
        else if (CheckCollision())
        {
            State.ActualHitTime = State.Time;
            State.ActualHitPosition = State.TargetPosition;
            State.Phase = SimulationPhase.Complete;
            return;
        }

        // Check for out of range (miss)
        if (IsProjectileOutOfRange())
        {
            State.Phase = SimulationPhase.Complete;
            return;
        }

        // Check for max duration
        if (State.Time >= _scenario.MaxDuration)
        {
            State.Phase = SimulationPhase.Complete;
        }
    }

    private void UpdateProjectile()
    {
        if (_scenario is null || !State.CastPosition.HasValue) return;

        if (_scenario.Skillshot.GetProjectileSpeed() is not double speed)
        {
            // Instant skillshot (like cone)
            State.ProjectilePosition = State.CastPosition;
            return;
        }

        var delay = _scenario.Skillshot.GetDelay();
        var flightTime = State.Time - delay;

        // Calculate projectile position along the path to cast position
        var direction = (State.CastPosition.Value - _scenario.CasterPosition).Normalize();
        var distance = speed * flightTime;

        State.ProjectilePosition = new Point2D(
            _scenario.CasterPosition.X + direction.X * distance,
            _scenario.CasterPosition.Y + direction.Y * distance);
    }

    /// <summary>
    /// Continuous collision for linear skillshots over one frame: closest
    /// approach between the moving missile front and the moving target center.
    /// Returns the earliest contact moment within the frame as a fraction (0..1).
    /// </summary>
    private bool CheckLinearContinuousCollision(
        double previousTime, Point2D previousTarget, out double contactFraction)
    {
        contactFraction = 1;

        if (_scenario is null || !State.CastPosition.HasValue) return false;
        if (_scenario.Skillshot is not Skillshot.Linear linear) return false;

        var caster = _scenario.CasterPosition;
        var toCast = State.CastPosition.Value - caster;
        if (toCast.Length < 1e-9) return false;

        var direction = toCast.Normalize();
        var delay = _scenario.Skillshot.GetDelay();

        // Missile front at frame start and end (clamped to the skillshot range)
        var previousTravel = Math.Clamp(linear.Speed * (previousTime - delay), 0, linear.Range);
        var currentTravel = Math.Clamp(linear.Speed * (State.Time - delay), 0, linear.Range);
        var previousFront = caster + direction.ScaleBy(previousTravel);
        var currentFront = caster + direction.ScaleBy(currentTravel);

        var effectiveRadius = _scenario.Skillshot.GetEffectiveRadius(_scenario.HitboxRadius);

        return LinearCollisionDetector.SweptContact(
            previousFront - previousTarget,
            currentFront - State.TargetPosition,
            effectiveRadius,
            out contactFraction);
    }

    private bool CheckCollision()
    {
        if (_scenario is null || !State.ProjectilePosition.HasValue) return false;

        // Use the prediction engine's collision validation
        var delay = _scenario.Skillshot.GetDelay();

        // For instant skillshots (cone), check at delay time
        if (_scenario.Skillshot.GetProjectileSpeed() is null)
        {
            // Cone: instant check at delay time
            if (State.Time >= delay && State.CastPosition.HasValue)
            {
                return _engine.ValidateHit(
                    _scenario.Skillshot,
                    _scenario.CasterPosition,
                    State.CastPosition.Value,
                    State.TargetPosition,
                    _scenario.HitboxRadius,
                    State.Time);
            }
            return false;
        }

        // For projectile skillshots, check collision along path
        if (State.CastPosition.HasValue)
        {
            return _engine.ValidateHit(
                _scenario.Skillshot,
                _scenario.CasterPosition,
                State.CastPosition.Value,
                State.TargetPosition,
                _scenario.HitboxRadius,
                State.Time);
        }

        return false;
    }

    private bool IsProjectileOutOfRange()
    {
        if (_scenario is null || !State.ProjectilePosition.HasValue) return false;

        var range = _scenario.Skillshot.GetMaxRange();
        var distance = _scenario.CasterPosition.DistanceTo(State.ProjectilePosition.Value);

        return distance > range + 50; // Buffer to account for hitbox radius
    }
}
