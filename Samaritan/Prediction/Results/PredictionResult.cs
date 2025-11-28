namespace Samaritan.Prediction.Results;

using Dunet;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Discriminated union representing the outcome of a prediction calculation.
/// </summary>
[Union]
public partial record PredictionResult
{
    /// <summary>
    /// Successful prediction with interception parameters.
    /// </summary>
    /// <param name="InterceptionTime">Time in seconds until interception.</param>
    /// <param name="CastPosition">Position to aim the skillshot.</param>
    /// <param name="PredictedPosition">Where the target will be at interception.</param>
    /// <param name="Confidence">Confidence level (0-1) of the prediction.</param>
    public partial record Hit(
        double InterceptionTime,
        Point2D CastPosition,
        Point2D PredictedPosition,
        double Confidence);

    /// <summary>
    /// Target is out of range for the skillshot.
    /// </summary>
    /// <param name="Distance">Actual distance to target.</param>
    /// <param name="MaxRange">Maximum skillshot range.</param>
    public partial record OutOfRange(double Distance, double MaxRange);

    /// <summary>
    /// No valid interception point found (target moving too fast, etc.).
    /// </summary>
    /// <param name="Reason">Explanation for why interception is impossible.</param>
    public partial record Unreachable(string Reason);
}

/// <summary>
/// Metadata about how a prediction was computed.
/// </summary>
public sealed record PredictionMetadata
{
    /// <summary>
    /// Name of the solver that produced this result.
    /// </summary>
    public required string SolverUsed { get; init; }

    /// <summary>
    /// Time taken to compute the prediction.
    /// </summary>
    public TimeSpan ComputationTime { get; init; }

    /// <summary>
    /// Number of iterations used (for numerical solvers).
    /// </summary>
    public int IterationsUsed { get; init; }

    /// <summary>
    /// Whether the result was retrieved from cache.
    /// </summary>
    public bool FromCache { get; init; }
}
