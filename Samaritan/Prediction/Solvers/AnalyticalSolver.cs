namespace Samaritan.Prediction.Solvers;

using MathNet.Numerics;
using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Configuration;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Analytical solver using closed-form quadratic solutions.
/// Best for constant velocity targets with linear/circular skillshots.
/// </summary>
public sealed class AnalyticalSolver : IInterceptionSolver
{
    private readonly PredictionConfig _config;

    /// <summary>
    /// Creates an analytical solver with the specified configuration.
    /// </summary>
    public AnalyticalSolver(PredictionConfig? config = null)
    {
        _config = config ?? PredictionConfig.Default;
    }

    /// <inheritdoc />
    public string Name => "Analytical";

    /// <inheritdoc />
    public bool CanSolve(Skillshot skillshot, MovementState target)
    {
        // Analytical solver works best for constant velocity movement
        return target switch
        {
            MovementState.Idle => true,
            MovementState.Walking => true,
            MovementState.Dashing => false, // Use numerical for dashes
            MovementState.Channeling c => c.Acceleration == 0, // Only if no acceleration
            _ => false
        };
    }

    /// <inheritdoc />
    public InterceptionSolution? Solve(
        Skillshot skillshot,
        Point2D sourcePosition,
        MovementState target,
        double hitboxRadius)
    {
        var targetPosition = target.GetPosition();
        var targetVelocity = target.GetVelocity();

        return skillshot.Match(
            linear: l => SolveLinear(sourcePosition, targetPosition, targetVelocity, l, hitboxRadius),
            circular: c => SolveCircular(sourcePosition, targetPosition, targetVelocity, c, hitboxRadius),
            cone: c => SolveCone(sourcePosition, targetPosition, targetVelocity, c),
            arc: _ => null, // Arc requires numerical solver
            rectangle: _ => null, // Rectangle requires numerical solver
            vectorRectangle: _ => null); // VectorRectangle requires numerical solver
    }

    private InterceptionSolution? SolveLinear(
        Point2D source,
        Point2D targetPos,
        Vector2D targetVel,
        Skillshot.Linear skillshot,
        double hitboxRadius)
    {
        var effectiveRadius = skillshot.Width / 2.0 + hitboxRadius;
        return SolveProjectileInterception(
            source, targetPos, targetVel,
            skillshot.Speed, skillshot.Delay, skillshot.Range,
            effectiveRadius);
    }

    private InterceptionSolution? SolveCircular(
        Point2D source,
        Point2D targetPos,
        Vector2D targetVel,
        Skillshot.Circular skillshot,
        double hitboxRadius)
    {
        var effectiveRadius = skillshot.Radius + hitboxRadius;
        return SolveProjectileInterception(
            source, targetPos, targetVel,
            skillshot.Speed, skillshot.Delay, skillshot.Range,
            effectiveRadius);
    }

    private InterceptionSolution? SolveCone(
        Point2D source,
        Point2D targetPos,
        Vector2D targetVel,
        Skillshot.Cone skillshot)
    {
        // Cone skillshots are instant with just a delay
        // Predict where target will be at delay time
        var predictedPos = targetPos + targetVel.ScaleBy(skillshot.Delay);
        var distance = source.DistanceTo(predictedPos);

        if (distance > skillshot.Range)
        {
            return null;
        }

        return InterceptionSolution.Exact(skillshot.Delay, predictedPos);
    }

    /// <summary>
    /// Solves the quadratic interception equation for projectile skillshots.
    /// </summary>
    /// <remarks>
    /// Given:
    ///   - Caster position C
    ///   - Target position P with velocity V
    ///   - Projectile speed s, cast delay d
    ///
    /// The equation |P + V*T - C| = s*(T - d) yields:
    ///   aT² + bT + c = 0
    ///
    /// where:
    ///   a = |V|² - s²
    ///   b = 2*(D·V + s²*d)
    ///   c = |D|² - s²*d²
    ///   D = P - C
    /// </remarks>
    private InterceptionSolution? SolveProjectileInterception(
        Point2D source,
        Point2D targetPos,
        Vector2D targetVel,
        double projectileSpeed,
        double delay,
        double range,
        double effectiveRadius)
    {
        var displacement = targetPos - source;
        var targetSpeedSq = targetVel.DotProduct(targetVel);
        var projectileSpeedSq = projectileSpeed * projectileSpeed;

        // Special case: stationary target
        if (targetSpeedSq < _config.Epsilon)
        {
            var distance = displacement.Length;
            if (distance > range + effectiveRadius)
            {
                return null;
            }

            var hitTime = delay + Math.Max(0, distance - effectiveRadius) / projectileSpeed;
            return InterceptionSolution.Exact(hitTime, targetPos);
        }

        // Quadratic coefficients
        var a = targetSpeedSq - projectileSpeedSq;
        var dDotV = displacement.DotProduct(targetVel);
        var b = 2.0 * (dDotV + projectileSpeedSq * delay - projectileSpeed * effectiveRadius);
        var c = displacement.DotProduct(displacement) - projectileSpeedSq * delay * delay
              + 2.0 * projectileSpeed * delay * effectiveRadius - effectiveRadius * effectiveRadius;

        var time = SolveQuadraticForMinPositiveRoot(a, b, c, delay);

        if (time is null || time.Value > _config.MaxPredictionTime)
        {
            return null;
        }

        // Calculate predicted position and verify range
        var predictedPos = targetPos + targetVel.ScaleBy(time.Value);
        var aimDistance = source.DistanceTo(predictedPos);

        if (aimDistance > range)
        {
            return null;
        }

        return InterceptionSolution.Exact(time.Value, predictedPos);
    }

    /// <summary>
    /// Solves ax² + bx + c = 0 and returns the minimum real root >= minValue.
    /// Uses MathNet.Numerics.FindRoots for robust numerical handling.
    /// </summary>
    private static double? SolveQuadraticForMinPositiveRoot(double a, double b, double c, double minValue)
    {
        // FindRoots.Quadratic expects coefficients in ascending order: c + b*x + a*x²
        var (root1, root2) = FindRoots.Quadratic(c, b, a);

        // Extract real roots (imaginary part ≈ 0)
        const double imagTolerance = 1e-9;
        var candidates = new List<double>(2);

        if (Math.Abs(root1.Imaginary) < imagTolerance)
            candidates.Add(root1.Real);
        if (Math.Abs(root2.Imaginary) < imagTolerance &&
            Math.Abs(root2.Real - root1.Real) > 1e-9) // Avoid duplicates
            candidates.Add(root2.Real);

        // Return minimum valid root
        double? result = null;
        foreach (var t in candidates)
        {
            if (t >= minValue && (result is null || t < result))
                result = t;
        }

        return result;
    }
}
