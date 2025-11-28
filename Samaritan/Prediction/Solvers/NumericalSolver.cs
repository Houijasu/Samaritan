namespace Samaritan.Prediction.Solvers;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Configuration;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Numerical solver using Newton-Raphson with bisection fallback.
/// Handles complex movement patterns like dashes and waypoint pathing.
/// </summary>
public sealed class NumericalSolver : IInterceptionSolver
{
    private readonly PredictionConfig _config;

    /// <summary>
    /// Creates a numerical solver with the specified configuration.
    /// </summary>
    public NumericalSolver(PredictionConfig? config = null)
    {
        _config = config ?? PredictionConfig.Default;
    }

    /// <inheritdoc />
    public string Name => "Numerical";

    /// <inheritdoc />
    public bool CanSolve(Skillshot skillshot, MovementState target)
    {
        // Numerical solver can handle all cases
        return true;
    }

    /// <inheritdoc />
    public InterceptionSolution? Solve(
        Skillshot skillshot,
        Point2D sourcePosition,
        MovementState target,
        double hitboxRadius)
    {
        return skillshot.Match(
            linear: l => SolveLinear(sourcePosition, target, l, hitboxRadius),
            circular: c => SolveCircular(sourcePosition, target, c, hitboxRadius),
            cone: c => SolveCone(sourcePosition, target, c),
            arc: a => SolveArc(sourcePosition, target, a, hitboxRadius),
            rectangle: r => SolveRectangle(sourcePosition, target, r, hitboxRadius),
            vectorRectangle: v => SolveVectorRectangle(sourcePosition, target, v, hitboxRadius));
    }

    private InterceptionSolution? SolveLinear(
        Point2D source,
        MovementState target,
        Skillshot.Linear skillshot,
        double hitboxRadius)
    {
        var effectiveRadius = skillshot.Width / 2.0 + hitboxRadius;
        return SolveWithNewtonRaphson(
            source, target,
            skillshot.Speed, skillshot.Delay, skillshot.Range,
            effectiveRadius);
    }

    private InterceptionSolution? SolveCircular(
        Point2D source,
        MovementState target,
        Skillshot.Circular skillshot,
        double hitboxRadius)
    {
        var effectiveRadius = skillshot.Radius + hitboxRadius;
        return SolveWithNewtonRaphson(
            source, target,
            skillshot.Speed, skillshot.Delay, skillshot.Range,
            effectiveRadius);
    }

    private InterceptionSolution? SolveCone(
        Point2D source,
        MovementState target,
        Skillshot.Cone skillshot)
    {
        // Cone is instant at delay, just predict position
        var predictedPos = target.PredictPosition(skillshot.Delay);
        var distance = source.DistanceTo(predictedPos);

        if (distance > skillshot.Range)
        {
            return null;
        }

        return InterceptionSolution.Exact(skillshot.Delay, predictedPos);
    }

    private InterceptionSolution? SolveArc(
        Point2D source,
        MovementState target,
        Skillshot.Arc skillshot,
        double hitboxRadius)
    {
        // Arc requires time-stepping simulation due to complex geometry
        return SolveWithTimeStepping(source, target, skillshot, hitboxRadius);
    }

    /// <summary>
    /// Solves for rectangle skillshots, accounting for width in the effective hit radius.
    /// The rectangle's width extends perpendicular to the aim direction.
    /// </summary>
    private InterceptionSolution? SolveRectangle(
        Point2D source,
        MovementState target,
        Skillshot.Rectangle skillshot,
        double hitboxRadius)
    {
        // Rectangle's effective hit radius is half its width plus target hitbox
        // This accounts for the perpendicular extent of the rectangle
        var effectiveRadius = skillshot.Width / 2.0 + hitboxRadius;

        return SolveWithNewtonRaphson(
            source, target,
            skillshot.Speed, skillshot.Delay, skillshot.Range,
            effectiveRadius);
    }

    /// <summary>
    /// Solves for vector-cast rectangles (like Viktor E, Rumble R).
    /// Returns the optimal predicted position - the vector should be cast
    /// from caster toward this position.
    /// </summary>
    private InterceptionSolution? SolveVectorRectangle(
        Point2D source,
        MovementState target,
        Skillshot.VectorRectangle skillshot,
        double hitboxRadius)
    {
        // For vector rectangles, we find where the target will be,
        // then the cast direction is from caster toward that point
        // The rectangle extends from a point within Range toward the target

        var effectiveWidth = skillshot.Width / 2.0 + hitboxRadius;

        // Find when the projectile would reach the target
        var initialPos = target.GetPosition();
        var distanceToTarget = source.DistanceTo(initialPos);

        // If target is out of total range (Range + MaxLength), no hit possible
        if (distanceToTarget > skillshot.Range + skillshot.MaxLength + effectiveWidth)
        {
            return null;
        }

        // Use Newton-Raphson to find interception time
        var result = SolveWithNewtonRaphson(
            source, target,
            skillshot.Speed, skillshot.Delay, skillshot.Range + skillshot.MaxLength,
            effectiveWidth);

        if (result is null)
        {
            return null;
        }

        // Verify the predicted position is reachable
        var predictedPos = result.Value.Position;
        var distToPredict = source.DistanceTo(predictedPos);

        // The start of the vector must be within Range of caster
        if (distToPredict > skillshot.Range + skillshot.MaxLength)
        {
            return null;
        }

        return result;
    }

    /// <summary>
    /// Newton-Raphson iterative solver.
    /// Solves f(t) = |P(t) - C| - s*(t - d) = 0
    /// </summary>
    private InterceptionSolution? SolveWithNewtonRaphson(
        Point2D source,
        MovementState target,
        double projectileSpeed,
        double delay,
        double range,
        double effectiveRadius)
    {
        // Initial guess based on current distance
        var initialPos = target.GetPosition();
        var initialDistance = source.DistanceTo(initialPos);
        var t = delay + initialDistance / projectileSpeed;

        for (var iteration = 0; iteration < _config.MaxNewtonIterations; iteration++)
        {
            var targetPos = target.PredictPosition(t);
            var targetVel = target switch
            {
                MovementState.Idle => new Vector2D(0, 0),
                MovementState.Walking w => w.Velocity,
                MovementState.Dashing d => GetDashVelocity(d, t),
                MovementState.Channeling c => c.Direction.ScaleBy(c.Speed),
                _ => new Vector2D(0, 0)
            };

            var diff = targetPos - source;
            var distance = diff.Length;

            // f(t) = distance - speed * (t - delay) - effectiveRadius
            var f = distance - projectileSpeed * (t - delay) - effectiveRadius;

            // Check convergence
            if (Math.Abs(f) < _config.ConvergenceTolerance)
            {
                if (t >= delay && source.DistanceTo(targetPos) <= range)
                {
                    return InterceptionSolution.Numerical(t, targetPos, iteration, _config.MaxNewtonIterations);
                }
            }

            // f'(t) = (diff · velocity) / distance - speed
            var fPrime = distance > _config.Epsilon
                ? diff.DotProduct(targetVel) / distance - projectileSpeed
                : -projectileSpeed;

            // Avoid division by zero
            if (Math.Abs(fPrime) < _config.Epsilon)
            {
                t += _config.CoarseTimeStep;
                continue;
            }

            var newT = t - f / fPrime;

            // Clamp to valid range
            newT = Math.Clamp(newT, delay, _config.MaxPredictionTime);

            // Check if converged
            if (Math.Abs(newT - t) < _config.ConvergenceTolerance)
            {
                var finalPos = target.PredictPosition(newT);
                if (source.DistanceTo(finalPos) <= range)
                {
                    return InterceptionSolution.Numerical(newT, finalPos, iteration, _config.MaxNewtonIterations);
                }
                return null;
            }

            t = newT;
        }

        // Newton-Raphson didn't converge, try bisection
        return SolveWithBisection(source, target, projectileSpeed, delay, range, effectiveRadius);
    }

    /// <summary>
    /// Bisection method fallback when Newton-Raphson fails.
    /// </summary>
    private InterceptionSolution? SolveWithBisection(
        Point2D source,
        MovementState target,
        double projectileSpeed,
        double delay,
        double range,
        double effectiveRadius)
    {
        // Find brackets where f changes sign
        var brackets = FindBrackets(source, target, projectileSpeed, delay, effectiveRadius);
        if (brackets is null)
        {
            return null;
        }

        var (lower, upper) = brackets.Value;

        // Bisection iterations
        for (var i = 0; i < _config.BinarySearchIterations; i++)
        {
            var mid = (lower + upper) / 2.0;
            var fMid = EvaluateFunction(source, target, projectileSpeed, delay, effectiveRadius, mid);

            if (Math.Abs(fMid) < _config.ConvergenceTolerance)
            {
                var pos = target.PredictPosition(mid);
                if (source.DistanceTo(pos) <= range)
                {
                    return InterceptionSolution.Numerical(mid, pos, i, _config.BinarySearchIterations);
                }
                return null;
            }

            var fLower = EvaluateFunction(source, target, projectileSpeed, delay, effectiveRadius, lower);

            if (fMid * fLower < 0)
            {
                upper = mid;
            }
            else
            {
                lower = mid;
            }
        }

        var finalT = (lower + upper) / 2.0;
        var finalPos = target.PredictPosition(finalT);

        if (source.DistanceTo(finalPos) <= range)
        {
            return InterceptionSolution.Numerical(finalT, finalPos, _config.BinarySearchIterations, _config.BinarySearchIterations);
        }

        return null;
    }

    /// <summary>
    /// Time-stepping simulation for complex skillshots (Arc).
    /// </summary>
    private InterceptionSolution? SolveWithTimeStepping(
        Point2D source,
        MovementState target,
        Skillshot.Arc skillshot,
        double hitboxRadius)
    {
        var effectiveWidth = skillshot.Width / 2.0 + hitboxRadius;

        // Coarse pass to find hit window
        double? firstHitTime = null;

        for (var t = (double)skillshot.Delay; t <= _config.MaxPredictionTime; t += _config.CoarseTimeStep)
        {
            var targetPos = target.PredictPosition(t);

            // Check if target is within arc range
            var distance = source.DistanceTo(targetPos);
            var inRange = distance <= skillshot.OuterRadius + effectiveWidth &&
                         distance >= skillshot.OuterRadius - skillshot.Width - effectiveWidth;

            if (inRange)
            {
                firstHitTime = t;
                break;
            }
        }

        if (firstHitTime is null)
        {
            return null;
        }

        // Refine with binary search
        var lower = Math.Max(skillshot.Delay, firstHitTime.Value - _config.CoarseTimeStep);
        var upper = firstHitTime.Value;

        for (var i = 0; i < _config.BinarySearchIterations; i++)
        {
            var mid = (lower + upper) / 2.0;
            var targetPos = target.PredictPosition(mid);
            var distance = source.DistanceTo(targetPos);

            var inRange = distance <= skillshot.OuterRadius + effectiveWidth &&
                         distance >= skillshot.OuterRadius - skillshot.Width - effectiveWidth;

            if (inRange)
            {
                upper = mid;
            }
            else
            {
                lower = mid;
            }
        }

        var finalT = (lower + upper) / 2.0;
        var finalPos = target.PredictPosition(finalT);

        return InterceptionSolution.Numerical(finalT, finalPos, _config.BinarySearchIterations, _config.BinarySearchIterations);
    }

    private (double Lower, double Upper)? FindBrackets(
        Point2D source,
        MovementState target,
        double speed,
        double delay,
        double effectiveRadius)
    {
        var prevT = delay;
        var prevF = EvaluateFunction(source, target, speed, delay, effectiveRadius, delay);

        for (var t = delay + _config.CoarseTimeStep; t <= _config.MaxPredictionTime; t += _config.CoarseTimeStep)
        {
            var f = EvaluateFunction(source, target, speed, delay, effectiveRadius, t);

            if (f * prevF < 0)
            {
                return (prevT, t);
            }

            prevT = t;
            prevF = f;
        }

        return null;
    }

    private double EvaluateFunction(
        Point2D source,
        MovementState target,
        double speed,
        double delay,
        double effectiveRadius,
        double t)
    {
        var targetPos = target.PredictPosition(t);
        var distance = source.DistanceTo(targetPos);
        return distance - speed * (t - delay) - effectiveRadius;
    }

    private static Vector2D GetDashVelocity(MovementState.Dashing dash, double currentTime)
    {
        var remainingTime = dash.Duration - dash.Elapsed;
        if (currentTime > remainingTime)
        {
            return new Vector2D(0, 0);
        }

        var direction = (dash.EndPosition - dash.StartPosition).Normalize();
        var distance = dash.StartPosition.DistanceTo(dash.EndPosition);
        return direction.ScaleBy(distance / dash.Duration);
    }
}
