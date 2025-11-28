namespace Samaritan.Prediction.Movement;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Configuration;

/// <summary>
/// Tracks target movement history and produces movement models for prediction.
/// </summary>
public sealed class MovementTracker
{
    private readonly PredictionConfig _config;
    private readonly MovementSample[] _samples;
    private int _head;
    private int _count;

    private MovementState _currentState;
    private IMovementModel? _currentModel;
    private double _lastUpdateTime;

    /// <summary>
    /// Creates a movement tracker with the specified configuration.
    /// </summary>
    /// <param name="config">Prediction configuration.</param>
    /// <param name="historyCapacity">Number of samples to keep in history.</param>
    public MovementTracker(PredictionConfig? config = null, int historyCapacity = 32)
    {
        _config = config ?? PredictionConfig.Default;
        _samples = new MovementSample[historyCapacity];
        _currentState = new MovementState.Idle(new Point2D(0, 0));
    }

    /// <summary>
    /// Current movement state of the target.
    /// </summary>
    public MovementState CurrentState => _currentState;

    /// <summary>
    /// Target's hitbox radius.
    /// </summary>
    public double HitboxRadius { get; set; }

    /// <summary>
    /// Number of samples in history.
    /// </summary>
    public int SampleCount => _count;

    /// <summary>
    /// Updates the tracker with a new position sample.
    /// </summary>
    /// <param name="position">Current position.</param>
    /// <param name="gameTime">Current game time in seconds.</param>
    public void Update(Point2D position, double gameTime)
    {
        var sample = new MovementSample(position, gameTime);
        AddSample(sample);
        _lastUpdateTime = gameTime;

        UpdateState();
        _currentModel = null; // Invalidate cached model
    }

    /// <summary>
    /// Notifies the tracker that the target started a dash.
    /// </summary>
    public void NotifyDash(
        Point2D startPosition,
        Point2D endPosition,
        double duration,
        double startTime,
        EaseType easeType = EaseType.Linear)
    {
        var elapsed = _lastUpdateTime - startTime;
        _currentState = new MovementState.Dashing(
            startPosition, endPosition, duration, elapsed, easeType);
        _currentModel = new AccelerationModel(
            startPosition, endPosition, duration, elapsed, easeType);
    }

    /// <summary>
    /// Notifies the tracker of known waypoints.
    /// </summary>
    public void NotifyWaypoints(IReadOnlyList<Point2D> waypoints, double moveSpeed)
    {
        if (waypoints.Count == 0)
        {
            return;
        }

        var currentPos = GetLatestPosition();
        var (velocity, _) = EstimateVelocity();

        _currentState = new MovementState.Walking(currentPos, velocity, waypoints[^1]);
        _currentModel = new WaypointModel(currentPos, waypoints, moveSpeed);
    }

    /// <summary>
    /// Gets the best movement model for prediction.
    /// </summary>
    public IMovementModel GetModel()
    {
        if (_currentModel is not null)
        {
            return _currentModel;
        }

        _currentModel = CreateModelFromState();
        return _currentModel;
    }

    /// <summary>
    /// Estimates the current velocity from position history.
    /// </summary>
    /// <returns>Velocity vector and confidence (0-1).</returns>
    public (Vector2D Velocity, double Confidence) EstimateVelocity()
    {
        if (_count < 2)
        {
            return (new Vector2D(0, 0), 0);
        }

        var sampleCount = Math.Min(5, _count);
        var totalVelocity = new Vector2D(0, 0);
        var totalWeight = 0.0;
        var velocityVariance = 0.0;
        var velocities = new List<Vector2D>();

        for (var i = 0; i < sampleCount - 1; i++)
        {
            var newer = GetSample(i);
            var older = GetSample(i + 1);
            var dt = newer.Timestamp - older.Timestamp;

            if (dt <= 0.001)
            {
                continue;
            }

            var velocity = (newer.Position - older.Position).ScaleBy(1.0 / dt);
            var weight = 1.0 / (i + 1); // More recent = higher weight

            velocities.Add(velocity);
            totalVelocity = totalVelocity + velocity.ScaleBy(weight);
            totalWeight += weight;
        }

        if (totalWeight < 0.001 || velocities.Count == 0)
        {
            return (new Vector2D(0, 0), 0);
        }

        var avgVelocity = totalVelocity.ScaleBy(1.0 / totalWeight);

        // Calculate variance for confidence
        foreach (var vel in velocities)
        {
            var diff = vel - avgVelocity;
            velocityVariance += diff.Length * diff.Length;
        }

        velocityVariance /= velocities.Count;

        // Confidence based on consistency
        var confidence = 1.0 / (1.0 + velocityVariance / 10000.0);
        return (avgVelocity, confidence);
    }

    /// <summary>
    /// Detects if the target recently changed direction.
    /// </summary>
    /// <param name="threshold">Angle threshold in degrees.</param>
    public bool DetectPathChange(double threshold = 15)
    {
        if (_count < 3)
        {
            return false;
        }

        var s0 = GetSample(0);
        var s1 = GetSample(1);
        var s2 = GetSample(2);

        var dir1 = s0.Position - s1.Position;
        var dir2 = s1.Position - s2.Position;

        if (dir1.Length < 1 || dir2.Length < 1)
        {
            return false;
        }

        var angleDegrees = dir1.AngleTo(dir2).Degrees;
        return Math.Abs(angleDegrees) > threshold;
    }

    private void AddSample(MovementSample sample)
    {
        _samples[_head] = sample;
        _head = (_head + 1) % _samples.Length;
        if (_count < _samples.Length)
        {
            _count++;
        }
    }

    private MovementSample GetSample(int indexFromNewest)
    {
        var index = (_head - 1 - indexFromNewest + _samples.Length) % _samples.Length;
        return _samples[index];
    }

    private Point2D GetLatestPosition()
    {
        return _count > 0 ? GetSample(0).Position : new Point2D(0, 0);
    }

    private void UpdateState()
    {
        if (_count < 2)
        {
            _currentState = new MovementState.Idle(GetLatestPosition());
            return;
        }

        var (velocity, confidence) = EstimateVelocity();
        var position = GetLatestPosition();

        // Check if stationary
        if (velocity.Length < 1)
        {
            _currentState = new MovementState.Idle(position);
            return;
        }

        // Check for path change
        if (DetectPathChange())
        {
            // Path changed, update velocity estimate
            (velocity, confidence) = EstimateVelocity();
        }

        // Keep as dashing if already dashing and not complete
        if (_currentState is MovementState.Dashing dash && dash.Elapsed < dash.Duration)
        {
            var newElapsed = dash.Elapsed + (_lastUpdateTime - GetSample(1).Timestamp);
            _currentState = dash with { Elapsed = newElapsed };
            return;
        }

        _currentState = new MovementState.Walking(position, velocity, null);
    }

    private IMovementModel CreateModelFromState()
    {
        return _currentState switch
        {
            MovementState.Idle idle => ConstantVelocityModel.Stationary(idle.Position),
            MovementState.Walking walking => new ConstantVelocityModel(walking.Position, walking.Velocity),
            MovementState.Dashing dashing => new AccelerationModel(
                dashing.StartPosition, dashing.EndPosition, dashing.Duration, dashing.Elapsed, dashing.EaseType),
            MovementState.Channeling channeling => new ConstantVelocityModel(
                channeling.Position, channeling.Direction.ScaleBy(channeling.Speed)),
            _ => ConstantVelocityModel.Stationary(new Point2D(0, 0))
        };
    }
}

/// <summary>
/// A single movement sample.
/// </summary>
/// <param name="Position">World position.</param>
/// <param name="Timestamp">Game time when sampled.</param>
public readonly record struct MovementSample(Point2D Position, double Timestamp);
