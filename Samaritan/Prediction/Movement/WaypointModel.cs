namespace Samaritan.Prediction.Movement;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Movement model for waypoint-based pathing (click-to-move).
/// Predicts movement along a sequence of waypoints.
/// </summary>
public sealed class WaypointModel : IMovementModel
{
    private readonly Point2D _currentPosition;
    private readonly IReadOnlyList<Point2D> _waypoints;
    private readonly double _moveSpeed;
    private readonly double _totalPathTime;

    /// <summary>
    /// Creates a waypoint movement model.
    /// </summary>
    /// <param name="currentPosition">Current position of the target.</param>
    /// <param name="waypoints">Ordered list of waypoints to follow.</param>
    /// <param name="moveSpeed">Movement speed in units per second.</param>
    public WaypointModel(
        Point2D currentPosition,
        IReadOnlyList<Point2D> waypoints,
        double moveSpeed)
    {
        _currentPosition = currentPosition;
        _waypoints = waypoints;
        _moveSpeed = Math.Max(1, moveSpeed);
        _totalPathTime = CalculateTotalPathTime();
    }

    /// <inheritdoc />
    public Point2D PredictPosition(double deltaTime)
    {
        if (deltaTime <= 0 || _waypoints.Count == 0)
        {
            return _currentPosition;
        }

        var position = _currentPosition;
        var remainingTime = deltaTime;
        var waypointIndex = 0;

        while (remainingTime > 0 && waypointIndex < _waypoints.Count)
        {
            var target = _waypoints[waypointIndex];
            var toTarget = target - position;
            var distance = toTarget.Length;

            if (distance < 0.001)
            {
                waypointIndex++;
                continue;
            }

            var timeToWaypoint = distance / _moveSpeed;

            if (timeToWaypoint <= remainingTime)
            {
                position = target;
                remainingTime -= timeToWaypoint;
                waypointIndex++;
            }
            else
            {
                var direction = toTarget.Normalize();
                position = position + direction.ScaleBy(_moveSpeed * remainingTime);
                remainingTime = 0;
            }
        }

        return position;
    }

    /// <inheritdoc />
    public Vector2D PredictVelocity(double deltaTime)
    {
        if (_waypoints.Count == 0)
        {
            return new Vector2D(0, 0);
        }

        // Find current segment and return velocity along it
        var position = PredictPosition(deltaTime);
        var nextPosition = PredictPosition(deltaTime + 0.001);

        var diff = nextPosition - position;
        if (diff.Length < 0.0001)
        {
            return new Vector2D(0, 0);
        }

        return diff.Normalize().ScaleBy(_moveSpeed);
    }

    /// <inheritdoc />
    public double GetConfidence(double deltaTime)
    {
        if (_totalPathTime <= 0 || _waypoints.Count == 0)
        {
            return 0.5;
        }

        // Confidence decreases with prediction time
        var progress = deltaTime / _totalPathTime;
        return Math.Max(0.2, 1 - progress * 0.6);
    }

    /// <inheritdoc />
    public double MaxReliableTime => _totalPathTime;

    private double CalculateTotalPathTime()
    {
        if (_waypoints.Count == 0)
        {
            return 0;
        }

        var totalTime = 0.0;
        var position = _currentPosition;

        foreach (var waypoint in _waypoints)
        {
            totalTime += position.DistanceTo(waypoint) / _moveSpeed;
            position = waypoint;
        }

        return totalTime;
    }
}
