namespace Samaritan.Prediction.Movement;

using Dunet;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Discriminated union representing the current movement state of a target.
/// </summary>
[Union]
public partial record MovementState
{
    /// <summary>
    /// Target is stationary (no movement input).
    /// </summary>
    /// <param name="Position">Current position.</param>
    public partial record Idle(Point2D Position);

    /// <summary>
    /// Target is moving at constant velocity toward a destination.
    /// </summary>
    /// <param name="Position">Current position.</param>
    /// <param name="Velocity">Current velocity vector (units/sec).</param>
    /// <param name="Destination">Click destination (if known).</param>
    public partial record Walking(Point2D Position, Vector2D Velocity, Point2D? Destination);

    /// <summary>
    /// Target is in a dash/blink with known parameters.
    /// </summary>
    /// <param name="StartPosition">Dash start position.</param>
    /// <param name="EndPosition">Dash end position.</param>
    /// <param name="Duration">Total dash duration in seconds.</param>
    /// <param name="Elapsed">Time elapsed since dash started.</param>
    /// <param name="EaseType">Easing function type for the dash.</param>
    public partial record Dashing(
        Point2D StartPosition,
        Point2D EndPosition,
        double Duration,
        double Elapsed,
        EaseType EaseType = EaseType.Linear);

    /// <summary>
    /// Target is in an unstoppable channel (e.g., Sion R).
    /// </summary>
    /// <param name="Position">Current position.</param>
    /// <param name="Direction">Movement direction (normalized).</param>
    /// <param name="Speed">Current speed (units/sec).</param>
    /// <param name="Acceleration">Speed change per second.</param>
    public partial record Channeling(Point2D Position, Vector2D Direction, double Speed, double Acceleration = 0);

    /// <summary>
    /// Target is following a path with multiple waypoints (e.g., right-click path in LoL).
    /// The target moves through each waypoint in sequence at constant speed.
    /// </summary>
    /// <param name="Waypoints">Ordered list of waypoints to traverse.</param>
    /// <param name="Speed">Movement speed (units/sec).</param>
    /// <param name="CurrentIndex">Index of current target waypoint (0 = moving to first waypoint).</param>
    /// <param name="ProgressOnSegment">Progress along current segment (0-1).</param>
    public partial record Pathing(
        IReadOnlyList<Point2D> Waypoints,
        double Speed,
        int CurrentIndex = 0,
        double ProgressOnSegment = 0);
}

/// <summary>
/// Easing functions for dash acceleration curves.
/// </summary>
public enum EaseType
{
    /// <summary>Constant speed throughout.</summary>
    Linear,

    /// <summary>Slow start, fast end.</summary>
    EaseIn,

    /// <summary>Fast start, slow end.</summary>
    EaseOut,

    /// <summary>Slow start and end.</summary>
    EaseInOut,

    /// <summary>Instant teleport (blink).</summary>
    Instant
}

/// <summary>
/// Extension methods for MovementState.
/// </summary>
public static class MovementStateExtensions
{
    /// <summary>
    /// Gets the current position from any movement state.
    /// </summary>
    public static Point2D GetPosition(this MovementState state)
    {
        return state switch
        {
            MovementState.Idle s => s.Position,
            MovementState.Walking s => s.Position,
            MovementState.Dashing s => GetDashPosition(s),
            MovementState.Channeling s => s.Position,
            MovementState.Pathing s => GetPathingPosition(s),
            _ => new Point2D(0, 0)
        };
    }

    /// <summary>
    /// Gets the current velocity from any movement state.
    /// </summary>
    public static Vector2D GetVelocity(this MovementState state)
    {
        return state switch
        {
            MovementState.Idle => new Vector2D(0, 0),
            MovementState.Walking s => s.Velocity,
            MovementState.Dashing s => GetDashVelocity(s),
            MovementState.Channeling s => s.Direction.ScaleBy(s.Speed),
            MovementState.Pathing s => GetPathingVelocity(s),
            _ => new Vector2D(0, 0)
        };
    }

    /// <summary>
    /// Predicts position at a future time delta.
    /// </summary>
    public static Point2D PredictPosition(this MovementState state, double deltaTime)
    {
        if (deltaTime <= 0)
        {
            return state.GetPosition();
        }

        return state switch
        {
            MovementState.Idle s => s.Position,
            MovementState.Walking s => s.Position + s.Velocity.ScaleBy(deltaTime),
            MovementState.Dashing s => PredictDashPosition(s, deltaTime),
            MovementState.Channeling s => PredictChannelingPosition(s, deltaTime),
            MovementState.Pathing s => PredictPathingPosition(s, deltaTime),
            _ => new Point2D(0, 0)
        };
    }

    /// <summary>
    /// Gets the velocity at a future time (important for pathing where direction changes).
    /// </summary>
    public static Vector2D GetVelocityAtTime(this MovementState state, double deltaTime)
    {
        if (state is MovementState.Pathing pathing)
        {
            return GetPathingVelocityAtTime(pathing, deltaTime);
        }
        return state.GetVelocity();
    }

    private static Point2D GetDashPosition(MovementState.Dashing dash)
    {
        var t = dash.Elapsed / dash.Duration;
        t = Math.Clamp(t, 0, 1);
        var easedT = ApplyEasing(t, dash.EaseType);
        return Lerp(dash.StartPosition, dash.EndPosition, easedT);
    }

    private static Vector2D GetDashVelocity(MovementState.Dashing dash)
    {
        if (dash.Elapsed >= dash.Duration)
        {
            return new Vector2D(0, 0);
        }

        var direction = (dash.EndPosition - dash.StartPosition).Normalize();
        var distance = dash.StartPosition.DistanceTo(dash.EndPosition);
        var baseSpeed = distance / dash.Duration;

        return direction.ScaleBy(baseSpeed);
    }

    private static Point2D PredictDashPosition(MovementState.Dashing dash, double deltaTime)
    {
        var newElapsed = dash.Elapsed + deltaTime;

        if (newElapsed >= dash.Duration)
        {
            return dash.EndPosition;
        }

        var t = newElapsed / dash.Duration;
        var easedT = ApplyEasing(t, dash.EaseType);
        return Lerp(dash.StartPosition, dash.EndPosition, easedT);
    }

    private static Point2D PredictChannelingPosition(MovementState.Channeling channel, double deltaTime)
    {
        // Calculate displacement using average velocity over time interval
        var newSpeed = channel.Speed + channel.Acceleration * deltaTime;
        var avgSpeed = (channel.Speed + newSpeed) / 2;
        return channel.Position + channel.Direction.ScaleBy(avgSpeed * deltaTime);
    }

    private static double ApplyEasing(double t, EaseType ease)
    {
        return ease switch
        {
            EaseType.Linear => t,
            EaseType.EaseIn => t * t,
            EaseType.EaseOut => 1 - (1 - t) * (1 - t),
            EaseType.EaseInOut => t < 0.5
                ? 2 * t * t
                : 1 - Math.Pow(-2 * t + 2, 2) / 2,
            EaseType.Instant => t > 0 ? 1 : 0,
            _ => t
        };
    }

    private static Point2D Lerp(Point2D a, Point2D b, double t)
    {
        var direction = b - a;
        return a + direction.ScaleBy(t);
    }

    #region Pathing Helpers

    private static Point2D GetPathingPosition(MovementState.Pathing pathing)
    {
        if (pathing.Waypoints.Count == 0)
            return new Point2D(0, 0);

        if (pathing.CurrentIndex >= pathing.Waypoints.Count)
            return pathing.Waypoints[^1];

        if (pathing.CurrentIndex == 0)
            return pathing.Waypoints[0];

        var segmentStart = pathing.Waypoints[pathing.CurrentIndex - 1];
        var segmentEnd = pathing.Waypoints[pathing.CurrentIndex];
        return Lerp(segmentStart, segmentEnd, pathing.ProgressOnSegment);
    }

    private static Vector2D GetPathingVelocity(MovementState.Pathing pathing)
    {
        if (pathing.Waypoints.Count < 2 || pathing.CurrentIndex >= pathing.Waypoints.Count)
            return new Vector2D(0, 0);

        var segmentStart = pathing.CurrentIndex > 0
            ? pathing.Waypoints[pathing.CurrentIndex - 1]
            : pathing.Waypoints[0];
        var segmentEnd = pathing.Waypoints[pathing.CurrentIndex];

        var direction = (segmentEnd - segmentStart).Normalize();
        return direction.ScaleBy(pathing.Speed);
    }

    private static Point2D PredictPathingPosition(MovementState.Pathing pathing, double deltaTime)
    {
        if (pathing.Waypoints.Count == 0)
            return new Point2D(0, 0);

        if (pathing.Waypoints.Count == 1 || pathing.CurrentIndex >= pathing.Waypoints.Count)
            return pathing.Waypoints[^1];

        var currentPos = GetPathingPosition(pathing);
        var remainingDistance = pathing.Speed * deltaTime;
        var segmentIndex = pathing.CurrentIndex > 0 ? pathing.CurrentIndex - 1 : 0;
        var posOnPath = currentPos;

        while (remainingDistance > 0 && segmentIndex < pathing.Waypoints.Count - 1)
        {
            var segmentEnd = pathing.Waypoints[segmentIndex + 1];
            var distToEnd = posOnPath.DistanceTo(segmentEnd);

            if (remainingDistance < distToEnd)
            {
                var direction = (segmentEnd - posOnPath).Normalize();
                return posOnPath + direction.ScaleBy(remainingDistance);
            }

            remainingDistance -= distToEnd;
            posOnPath = segmentEnd;
            segmentIndex++;
        }

        return pathing.Waypoints[^1];
    }

    private static Vector2D GetPathingVelocityAtTime(MovementState.Pathing pathing, double deltaTime)
    {
        if (pathing.Waypoints.Count < 2)
            return new Vector2D(0, 0);

        var currentPos = GetPathingPosition(pathing);
        var remainingDistance = pathing.Speed * deltaTime;
        var segmentIndex = pathing.CurrentIndex > 0 ? pathing.CurrentIndex - 1 : 0;
        var posOnPath = currentPos;

        while (remainingDistance > 0 && segmentIndex < pathing.Waypoints.Count - 1)
        {
            var segmentEnd = pathing.Waypoints[segmentIndex + 1];
            var distToEnd = posOnPath.DistanceTo(segmentEnd);

            if (remainingDistance < distToEnd)
            {
                var direction = (segmentEnd - posOnPath).Normalize();
                return direction.ScaleBy(pathing.Speed);
            }

            remainingDistance -= distToEnd;
            posOnPath = segmentEnd;
            segmentIndex++;
        }

        return new Vector2D(0, 0);
    }

    /// <summary>
    /// Gets detailed path information for advanced solvers.
    /// Returns segments with their start/end positions and time ranges.
    /// </summary>
    public static IEnumerable<PathSegment> GetPathSegments(this MovementState.Pathing pathing)
    {
        if (pathing.Waypoints.Count < 2)
            yield break;

        var currentPos = GetPathingPosition(pathing);
        var accumulatedTime = 0.0;

        var firstEnd = pathing.Waypoints[pathing.CurrentIndex];
        var firstDist = currentPos.DistanceTo(firstEnd);
        
        if (firstDist > 1e-4)
        {
            var firstDuration = firstDist / pathing.Speed;

            yield return new PathSegment(
                currentPos,
                firstEnd,
                accumulatedTime,
                accumulatedTime + firstDuration,
                pathing.Speed);

            accumulatedTime += firstDuration;
        }

        for (var i = pathing.CurrentIndex; i < pathing.Waypoints.Count - 1; i++)
        {
            var start = pathing.Waypoints[i];
            var end = pathing.Waypoints[i + 1];
            var distance = start.DistanceTo(end);
            var duration = distance / pathing.Speed;

            yield return new PathSegment(
                start,
                end,
                accumulatedTime,
                accumulatedTime + duration,
                pathing.Speed);

            accumulatedTime += duration;
        }
    }

    #endregion
}

/// <summary>
/// Represents a segment of a movement path for interception calculations.
/// </summary>
public readonly record struct PathSegment(
    Point2D Start,
    Point2D End,
    double StartTime,
    double EndTime,
    double Speed)
{
    /// <summary>
    /// Direction of movement along this segment.
    /// </summary>
    public Vector2D Direction => (End - Start).Normalize();

    /// <summary>
    /// Velocity vector for this segment.
    /// </summary>
    public Vector2D Velocity => Direction.ScaleBy(Speed);

    /// <summary>
    /// Length of this segment.
    /// </summary>
    public double Length => Start.DistanceTo(End);

    /// <summary>
    /// Duration to traverse this segment.
    /// </summary>
    public double Duration => EndTime - StartTime;

    /// <summary>
    /// Gets position along segment at given time (relative to segment start).
    /// </summary>
    public Point2D GetPositionAtTime(double t)
    {
        var localT = Math.Clamp(t - StartTime, 0, Duration);
        return Start + Direction.ScaleBy(Speed * localT);
    }
}
