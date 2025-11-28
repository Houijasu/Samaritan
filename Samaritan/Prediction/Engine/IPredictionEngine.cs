namespace Samaritan.Prediction.Engine;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Interface for the main prediction engine.
/// </summary>
public interface IPredictionEngine
{
    /// <summary>
    /// Predicts the optimal aim point for hitting a moving target.
    /// </summary>
    /// <param name="skillshot">The skillshot to predict.</param>
    /// <param name="casterPosition">Position of the caster.</param>
    /// <param name="target">Target's movement tracker.</param>
    /// <returns>Prediction result.</returns>
    PredictionResult Predict(
        Skillshot skillshot,
        Point2D casterPosition,
        MovementTracker target);

    /// <summary>
    /// Predicts aim points for multiple targets.
    /// </summary>
    /// <param name="skillshot">The skillshot to predict.</param>
    /// <param name="casterPosition">Position of the caster.</param>
    /// <param name="targets">Target movement trackers.</param>
    /// <returns>Prediction results for each target.</returns>
    IReadOnlyList<PredictionResult> PredictMultiple(
        Skillshot skillshot,
        Point2D casterPosition,
        IEnumerable<MovementTracker> targets);

    /// <summary>
    /// Predicts using a specific movement state (without tracking).
    /// </summary>
    /// <param name="skillshot">The skillshot to predict.</param>
    /// <param name="casterPosition">Position of the caster.</param>
    /// <param name="targetState">Target's current movement state.</param>
    /// <param name="hitboxRadius">Target's hitbox radius.</param>
    /// <returns>Prediction result.</returns>
    PredictionResult PredictFromState(
        Skillshot skillshot,
        Point2D casterPosition,
        MovementState targetState,
        double hitboxRadius);

    /// <summary>
    /// Validates whether a skillshot aimed at a position will hit a target.
    /// </summary>
    /// <param name="skillshot">The skillshot to validate.</param>
    /// <param name="casterPosition">Position where the skillshot is cast from.</param>
    /// <param name="aimPosition">Position the skillshot is aimed at.</param>
    /// <param name="targetPosition">Current position of the target.</param>
    /// <param name="hitboxRadius">Target's hitbox radius.</param>
    /// <param name="timeElapsed">Time since skillshot was cast.</param>
    /// <returns>True if the skillshot will hit.</returns>
    bool ValidateHit(
        Skillshot skillshot,
        Point2D casterPosition,
        Point2D aimPosition,
        Point2D targetPosition,
        double hitboxRadius,
        double timeElapsed);
}
