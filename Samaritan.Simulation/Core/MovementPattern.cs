namespace Samaritan.Simulation.Core;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Defines how a target moves during simulation.
/// </summary>
public abstract record MovementPattern
{
    /// <summary>
    /// Get the position at a given time.
    /// </summary>
    public abstract Point2D GetPosition(double time);

    /// <summary>
    /// Get the velocity at a given time.
    /// </summary>
    public abstract Vector2D GetVelocity(double time);

    /// <summary>
    /// Target stays at a fixed position.
    /// </summary>
    public sealed record Stationary(Point2D Position) : MovementPattern
    {
        public override Point2D GetPosition(double time) => Position;
        public override Vector2D GetVelocity(double time) => new(0, 0);
    }

    /// <summary>
    /// Target moves in a straight line at constant velocity.
    /// </summary>
    public sealed record Linear(Point2D Start, Vector2D Velocity, double Duration) : MovementPattern
    {
        public override Point2D GetPosition(double time)
        {
            var t = Math.Min(time, Duration);
            return new Point2D(
                Start.X + Velocity.X * t,
                Start.Y + Velocity.Y * t);
        }

        public override Vector2D GetVelocity(double time)
        {
            return time < Duration ? Velocity : new Vector2D(0, 0);
        }
    }

    /// <summary>
    /// Target follows a sequence of waypoints at constant speed.
    /// </summary>
    public sealed record Waypoints(Point2D[] Points, double Speed) : MovementPattern
    {
        public override Point2D GetPosition(double time)
        {
            if (Points.Length == 0) return new Point2D(0, 0);
            if (Points.Length == 1) return Points[0];

            var totalDistance = 0.0;
            var targetDistance = Speed * time;

            for (var i = 0; i < Points.Length - 1; i++)
            {
                var segmentLength = Points[i].DistanceTo(Points[i + 1]);
                if (totalDistance + segmentLength >= targetDistance)
                {
                    var segmentProgress = (targetDistance - totalDistance) / segmentLength;
                    return new Point2D(
                        Points[i].X + (Points[i + 1].X - Points[i].X) * segmentProgress,
                        Points[i].Y + (Points[i + 1].Y - Points[i].Y) * segmentProgress);
                }
                totalDistance += segmentLength;
            }

            return Points[^1];
        }

        public override Vector2D GetVelocity(double time)
        {
            if (Points.Length < 2) return new Vector2D(0, 0);

            var totalDistance = 0.0;
            var targetDistance = Speed * time;

            for (var i = 0; i < Points.Length - 1; i++)
            {
                var segmentLength = Points[i].DistanceTo(Points[i + 1]);
                if (totalDistance + segmentLength >= targetDistance)
                {
                    var direction = (Points[i + 1] - Points[i]).Normalize();
                    return direction.ScaleBy(Speed);
                }
                totalDistance += segmentLength;
            }

            return new Vector2D(0, 0);
        }
    }
}
