namespace Samaritan.Simulation.Core;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;
using Samaritan.Simulation.Metrics;
using Samaritan.Simulation.Scenarios;

/// <summary>
/// Runs the skillshot prediction simulation.
/// </summary>
public class SimulationRunner
{
    private readonly PredictionEngine _engine = new();
    private readonly SimulationLogger _logger = new();
    private Scenario? _scenario;

    public SimulationState State { get; } = new();

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
        }

        // Get prediction using the movement state
        State.Prediction = _engine.PredictFromState(
            _scenario.Skillshot,
            _scenario.CasterPosition,
            movementState,
            _scenario.HitboxRadius);

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
        var delay = GetSkillshotDelay(_scenario.Skillshot);

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

        State.Time += deltaTime;

        // Update target position
        State.TargetPosition = _scenario.TargetMovement.GetPosition(State.Time);
        State.TargetVelocity = _scenario.TargetMovement.GetVelocity(State.Time);

        // Update projectile position
        UpdateProjectile();

        // Check for collision
        if (CheckCollision())
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

        var speed = GetSkillshotSpeed(_scenario.Skillshot);
        if (speed <= 0)
        {
            // Instant skillshot (like cone)
            State.ProjectilePosition = State.CastPosition;
            return;
        }

        var delay = GetSkillshotDelay(_scenario.Skillshot);
        var flightTime = State.Time - delay;

        // Calculate projectile position along the path to cast position
        var direction = (State.CastPosition.Value - _scenario.CasterPosition).Normalize();
        var distance = speed * flightTime;

        State.ProjectilePosition = new Point2D(
            _scenario.CasterPosition.X + direction.X * distance,
            _scenario.CasterPosition.Y + direction.Y * distance);
    }

    private bool CheckCollision()
    {
        if (_scenario is null || !State.ProjectilePosition.HasValue) return false;

        // Use the prediction engine's collision validation
        var delay = GetSkillshotDelay(_scenario.Skillshot);

        // For instant skillshots (cone), check at delay time
        var speed = GetSkillshotSpeed(_scenario.Skillshot);
        if (speed <= 0)
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

        var range = GetSkillshotRange(_scenario.Skillshot);
        var distance = _scenario.CasterPosition.DistanceTo(State.ProjectilePosition.Value);

        return distance > range + 50; // Buffer to account for hitbox radius
    }

    private static double GetSkillshotDelay(Skillshot skillshot)
    {
        return skillshot.Match(
            linear: l => l.Delay,
            circular: c => c.Delay,
            cone: c => c.Delay,
            arc: a => a.Delay,
            rectangle: r => r.Delay,
            vectorRectangle: v => v.Delay);
    }

    private static double GetSkillshotSpeed(Skillshot skillshot)
    {
        return skillshot.Match(
            linear: l => l.Speed,
            circular: c => c.Speed,
            cone: _ => 0, // Instant
            arc: a => a.Speed,
            rectangle: r => r.Speed,
            vectorRectangle: v => v.Speed);
    }

    private static double GetSkillshotRange(Skillshot skillshot)
    {
        return skillshot.Match(
            linear: l => l.Range,
            circular: c => c.Range,
            cone: c => c.Range,
            arc: a => a.OuterRadius,
            rectangle: r => r.Range,
            vectorRectangle: v => v.Range + v.MaxLength);
    }
}
