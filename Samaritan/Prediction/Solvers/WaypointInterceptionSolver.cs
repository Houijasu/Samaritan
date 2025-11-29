namespace Samaritan.Prediction.Solvers;

using MathNet.Numerics;
using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Configuration;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Solver specialized for targets following waypoint paths.
/// Iterates through path segments and finds the earliest valid interception.
/// Based on segment-by-segment interception algorithm.
/// </summary>
public sealed class WaypointInterceptionSolver : IInterceptionSolver
{
    private readonly PredictionConfig _config;

    /// <summary>
    /// Creates a waypoint interception solver.
    /// </summary>
    /// <param name="config">Prediction configuration (uses default if null).</param>
    public WaypointInterceptionSolver(PredictionConfig? config = null)
    {
        _config = config ?? PredictionConfig.Default;
    }

    /// <inheritdoc />
    public string Name => "Waypoint";

    /// <inheritdoc />
    public bool CanSolve(Skillshot skillshot, MovementState target)
    {
        return target is MovementState.Pathing;
    }

    /// <inheritdoc />
    public InterceptionSolution? Solve(
        Skillshot skillshot,
        Point2D sourcePosition,
        MovementState target,
        double hitboxRadius)
    {
        if (target is not MovementState.Pathing pathing)
            return null;

        var segments = pathing.GetPathSegments().ToList();
        if (segments.Count == 0)
            return null;

        // Get skillshot parameters
        var (delay, speed, range) = GetSkillshotParams(skillshot);
        var targetSpeed = segments[0].Velocity.Length;

        // Cut path by (delay * targetSpeed - hitbox) to account for cast delay and trailing edge
        var cutLength = delay * targetSpeed - hitboxRadius;
        var cutSegments = CutPath(segments, cutLength);

        if (cutSegments.Count == 0)
            return null;

        // Iterate through segments to find interception point
        double tTotal = 0;
        const double Epsilon = 1e-4;
        var sqrSpeed = speed * speed;

        foreach (var segment in cutSegments)
        {
            var diff = segment.Start - sourcePosition;
            var velocity = segment.Velocity;
            var duration = segment.Duration;

            // Quadratic formula: a = v² - p², b = 2(diff·v - p²·tTotal), c = diff² - p²·tTotal²
            var a = velocity.DotProduct(velocity) - sqrSpeed;
            var b = 2.0 * (diff.DotProduct(velocity) - sqrSpeed * tTotal);
            var c = diff.DotProduct(diff) - sqrSpeed * tTotal * tTotal;

            // Use FindRoots.Quadratic for robust numerical handling
            var (root1, root2) = FindRoots.Quadratic(c, b, a);

            const double ImagTol = 1e-9;
            var tIntercept = double.MaxValue;

            if (Math.Abs(root1.Imaginary) < ImagTol && root1.Real >= 0 && root1.Real <= duration + Epsilon)
                tIntercept = Math.Min(tIntercept, root1.Real);
            if (Math.Abs(root2.Imaginary) < ImagTol && root2.Real >= 0 && root2.Real <= duration + Epsilon)
                tIntercept = Math.Min(tIntercept, root2.Real);

            if (tIntercept < double.MaxValue)
            {
                var aimPoint = segment.Start + velocity.ScaleBy(tIntercept);

                if (sourcePosition.DistanceTo(aimPoint) <= range)
                {
                    var totalTime = delay + tTotal + tIntercept;
                    return InterceptionSolution.Exact(totalTime, aimPoint);
                }
            }

            tTotal += duration;
        }

        // No valid interception found
        return null;
    }

    /// <summary>
    /// Cuts a path by the specified distance.
    /// Positive distance advances along the path, negative extends backwards.
    /// </summary>
    private static List<PathSegment> CutPath(List<PathSegment> segments, double distance)
    {
        var result = new List<PathSegment>();

        if (segments.Count == 0)
            return result;

        // If distance is negative, extend the first segment backwards
        if (distance < 0)
        {
            var first = segments[0];
            var extendedStart = first.Start + first.Direction.ScaleBy(distance);
            var newStartTime = first.StartTime + distance / first.Speed;
            var newFirst = new PathSegment(extendedStart, first.End, newStartTime, first.EndTime, first.Speed);
            result.Add(newFirst);

            for (var i = 1; i < segments.Count; i++)
                result.Add(segments[i]);

            return result;
        }

        // Positive distance: advance along path
        var remaining = distance;
        for (var i = 0; i < segments.Count; i++)
        {
            var seg = segments[i];
            var segLength = seg.Length;

            if (remaining < segLength)
            {
                var newStart = seg.Start + seg.Direction.ScaleBy(remaining);
                var newStartTime = seg.StartTime + remaining / seg.Speed;
                var newSegment = new PathSegment(newStart, seg.End, newStartTime, seg.EndTime, seg.Speed);
                result.Add(newSegment);

                for (var j = i + 1; j < segments.Count; j++)
                    result.Add(segments[j]);

                return result;
            }

            remaining -= segLength;
        }

        // Distance exceeds path - return last point as zero-length segment
        var last = segments[^1];
        result.Add(new PathSegment(last.End, last.End, last.EndTime, last.EndTime, last.Speed));
        return result;
    }

    private static (double Delay, double Speed, double Range) GetSkillshotParams(Skillshot skillshot)
    {
        return skillshot.Match(
            linear: l => ((double)l.Delay, (double)l.Speed, (double)l.Range),
            circular: c => ((double)c.Delay, (double)c.Speed, (double)c.Range),
            cone: c => ((double)c.Delay, 0.0, (double)c.Range),
            arc: a => ((double)a.Delay, (double)a.Speed, (double)a.OuterRadius),
            rectangle: r => ((double)r.Delay, (double)r.Speed, (double)r.Range),
            vectorRectangle: v => ((double)v.Delay, (double)v.Speed, (double)(v.Range + v.MaxLength)));
    }
}
