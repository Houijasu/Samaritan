namespace Samaritan.Simulation.Scenarios;

using MathNet.Spatial.Euclidean;

using Samaritan.Simulation.Core;

/// <summary>
/// Defines a simulation scenario for testing prediction accuracy.
/// </summary>
public record Scenario(
    string Name,
    Skillshot Skillshot,
    Point2D CasterPosition,
    MovementPattern TargetMovement,
    double HitboxRadius)
{
    /// <summary>
    /// Maximum simulation time in seconds.
    /// </summary>
    public double MaxDuration { get; init; } = 5.0;
}
