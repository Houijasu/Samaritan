namespace Samaritan.Prediction.Solvers;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Configuration;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Hybrid solver that combines analytical, waypoint, and numerical approaches.
/// Selects the best solver based on target movement type.
/// </summary>
public sealed class HybridSolver : IInterceptionSolver
{
    private readonly AnalyticalSolver _analyticalSolver;
    private readonly WaypointInterceptionSolver _waypointSolver;
    private readonly NumericalSolver _numericalSolver;

    /// <summary>
    /// Creates a hybrid solver with the specified configuration.
    /// </summary>
    public HybridSolver(PredictionConfig? config = null)
    {
        var cfg = config ?? PredictionConfig.Default;
        _analyticalSolver = new AnalyticalSolver(cfg);
        _waypointSolver = new WaypointInterceptionSolver(cfg);
        _numericalSolver = new NumericalSolver(cfg);
    }

    /// <inheritdoc />
    public string Name => "Hybrid";

    /// <inheritdoc />
    public bool CanSolve(Skillshot skillshot, MovementState target) => true;

    /// <inheritdoc />
    public InterceptionSolution? Solve(
        Skillshot skillshot,
        Point2D sourcePosition,
        MovementState target,
        double hitboxRadius)
    {
        // Use waypoint solver for pathing targets
        if (_waypointSolver.CanSolve(skillshot, target))
        {
            var waypointResult = _waypointSolver.Solve(
                skillshot, sourcePosition, target, hitboxRadius);

            if (waypointResult is not null)
            {
                return waypointResult;
            }
        }

        // Try analytical solver for constant velocity cases
        if (_analyticalSolver.CanSolve(skillshot, target))
        {
            var analyticalResult = _analyticalSolver.Solve(
                skillshot, sourcePosition, target, hitboxRadius);

            if (analyticalResult is not null)
            {
                return analyticalResult;
            }
        }

        // Fall back to numerical solver
        return _numericalSolver.Solve(skillshot, sourcePosition, target, hitboxRadius);
    }
}
