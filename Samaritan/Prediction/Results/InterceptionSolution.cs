namespace Samaritan.Prediction.Results;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Internal result from interception solvers.
/// </summary>
/// <param name="Time">Interception time in seconds.</param>
/// <param name="Position">Predicted interception position.</param>
/// <param name="Confidence">Confidence level (0-1).</param>
public readonly record struct InterceptionSolution(
    double Time,
    Point2D Position,
    double Confidence)
{
    /// <summary>
    /// Creates a solution with full confidence.
    /// </summary>
    public static InterceptionSolution Exact(double time, Point2D position)
        => new(time, position, 1.0);

    /// <summary>
    /// Creates a solution from a numerical method with computed confidence.
    /// </summary>
    public static InterceptionSolution Numerical(double time, Point2D position, int iterations, int maxIterations)
        => new(time, position, 1.0 - (double)iterations / maxIterations * 0.3);
}
