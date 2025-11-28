namespace Samaritan.Prediction.Solvers;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Strategy interface for skillshot interception solvers.
/// </summary>
public interface IInterceptionSolver
{
    /// <summary>
    /// Name of this solver for diagnostics.
    /// </summary>
    string Name { get; }

    /// <summary>
    /// Determines if this solver can handle the given skillshot and target combination.
    /// </summary>
    /// <param name="skillshot">The skillshot to predict.</param>
    /// <param name="target">The target's movement state.</param>
    /// <returns>True if this solver can produce a solution.</returns>
    bool CanSolve(Skillshot skillshot, MovementState target);

    /// <summary>
    /// Calculates the interception solution for hitting a moving target.
    /// </summary>
    /// <param name="skillshot">The skillshot to predict.</param>
    /// <param name="sourcePosition">Caster's position.</param>
    /// <param name="target">The target's movement state.</param>
    /// <param name="hitboxRadius">Target's hitbox radius.</param>
    /// <returns>The interception solution, or null if no solution exists.</returns>
    InterceptionSolution? Solve(
        Skillshot skillshot,
        Point2D sourcePosition,
        MovementState target,
        double hitboxRadius);
}
