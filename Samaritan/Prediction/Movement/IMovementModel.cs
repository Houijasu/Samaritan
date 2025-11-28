namespace Samaritan.Prediction.Movement;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Interface for movement prediction models.
/// Each model predicts target position at future times.
/// </summary>
public interface IMovementModel
{
    /// <summary>
    /// Predicts position at a given time offset from now.
    /// </summary>
    /// <param name="deltaTime">Time in the future (seconds).</param>
    /// <returns>Predicted position.</returns>
    Point2D PredictPosition(double deltaTime);

    /// <summary>
    /// Predicts velocity at a given time offset.
    /// </summary>
    /// <param name="deltaTime">Time in the future (seconds).</param>
    /// <returns>Predicted velocity.</returns>
    Vector2D PredictVelocity(double deltaTime);

    /// <summary>
    /// Gets confidence in prediction at given time (0-1).
    /// Confidence typically decreases with time.
    /// </summary>
    /// <param name="deltaTime">Time in the future (seconds).</param>
    /// <returns>Confidence value between 0 and 1.</returns>
    double GetConfidence(double deltaTime);

    /// <summary>
    /// Maximum reliable prediction time for this model.
    /// </summary>
    double MaxReliableTime { get; }
}
