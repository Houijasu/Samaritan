namespace Samaritan.Prediction.Movement;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Simple constant velocity movement model.
/// Assumes target continues at current velocity indefinitely.
/// </summary>
public sealed class ConstantVelocityModel : IMovementModel
{
    private readonly Point2D _position;
    private readonly Vector2D _velocity;
    private readonly double _baseConfidence;

    /// <summary>
    /// Creates a constant velocity model.
    /// </summary>
    /// <param name="position">Current position.</param>
    /// <param name="velocity">Current velocity.</param>
    /// <param name="confidence">Base confidence level (0-1).</param>
    public ConstantVelocityModel(Point2D position, Vector2D velocity, double confidence = 1.0)
    {
        _position = position;
        _velocity = velocity;
        _baseConfidence = Math.Clamp(confidence, 0, 1);
    }

    /// <inheritdoc />
    public Point2D PredictPosition(double deltaTime)
        => _position + _velocity.ScaleBy(Math.Max(0, deltaTime));

    /// <inheritdoc />
    public Vector2D PredictVelocity(double deltaTime)
        => _velocity;

    /// <inheritdoc />
    public double GetConfidence(double deltaTime)
    {
        if (deltaTime <= 0)
        {
            return _baseConfidence;
        }

        // Confidence decays exponentially with time
        // Half-life of 0.5 seconds
        var decay = Math.Exp(-1.386 * deltaTime);
        return _baseConfidence * decay;
    }

    /// <inheritdoc />
    public double MaxReliableTime => 1.5;

    /// <summary>
    /// Creates a model for a stationary target.
    /// </summary>
    public static ConstantVelocityModel Stationary(Point2D position)
        => new(position, new Vector2D(0, 0), 0.95);
}
