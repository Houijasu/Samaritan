namespace Samaritan.Prediction.Movement;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Movement model for dashes and other accelerating movements.
/// Supports various easing functions for realistic dash behavior.
/// </summary>
public sealed class AccelerationModel : IMovementModel
{
    private readonly Point2D _startPosition;
    private readonly Point2D _endPosition;
    private readonly double _duration;
    private readonly double _elapsedTime;
    private readonly EaseType _easeType;

    /// <summary>
    /// Creates an acceleration model for a dash.
    /// </summary>
    /// <param name="startPosition">Dash starting position.</param>
    /// <param name="endPosition">Dash ending position.</param>
    /// <param name="duration">Total dash duration in seconds.</param>
    /// <param name="elapsedTime">Time already elapsed since dash started.</param>
    /// <param name="easeType">Easing function for the dash.</param>
    public AccelerationModel(
        Point2D startPosition,
        Point2D endPosition,
        double duration,
        double elapsedTime,
        EaseType easeType = EaseType.Linear)
    {
        _startPosition = startPosition;
        _endPosition = endPosition;
        _duration = Math.Max(0.001, duration);
        _elapsedTime = Math.Max(0, elapsedTime);
        _easeType = easeType;
    }

    /// <inheritdoc />
    public Point2D PredictPosition(double deltaTime)
    {
        var totalTime = _elapsedTime + Math.Max(0, deltaTime);

        // Dash completed - return end position
        if (totalTime >= _duration)
        {
            return _endPosition;
        }

        var t = totalTime / _duration;
        var easedT = ApplyEasing(t);

        return Lerp(_startPosition, _endPosition, easedT);
    }

    /// <inheritdoc />
    public Vector2D PredictVelocity(double deltaTime)
    {
        var totalTime = _elapsedTime + Math.Max(0, deltaTime);

        if (totalTime >= _duration)
        {
            return new Vector2D(0, 0);
        }

        // Numerical derivative for velocity
        const double h = 0.001;
        var p1 = PredictPosition(deltaTime);
        var p2 = PredictPosition(deltaTime + h);
        return (p2 - p1).ScaleBy(1.0 / h);
    }

    /// <inheritdoc />
    public double GetConfidence(double deltaTime)
    {
        var totalTime = _elapsedTime + Math.Max(0, deltaTime);

        // High confidence during dash
        if (totalTime < _duration)
        {
            return 0.95;
        }

        // After dash completes, confidence drops rapidly
        var overtime = totalTime - _duration;
        return 0.5 * Math.Exp(-2 * overtime);
    }

    /// <inheritdoc />
    public double MaxReliableTime => (_duration - _elapsedTime) + 0.2;

    /// <summary>
    /// Remaining time until dash completes.
    /// </summary>
    public double RemainingTime => Math.Max(0, _duration - _elapsedTime);

    private double ApplyEasing(double t)
    {
        return _easeType switch
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
}
