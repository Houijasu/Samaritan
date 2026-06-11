namespace Samaritan.Tests.Solvers;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Solvers;

public class WaypointInterceptionSolverTests
{
    private readonly WaypointInterceptionSolver _solver = new();

    [Fact]
    public void Solve_ConeVsPathingTarget_PredictsPositionAtDelay()
    {
        var cone = new Skillshot.Cone(Delay: 0.5f, Angle: 60, Range: 600);
        var source = new Point2D(0, 0);
        var pathing = new MovementState.Pathing(
            Waypoints: new[] { new Point2D(300, 0), new Point2D(400, 100) },
            Speed: 350,
            CurrentIndex: 1,
            ProgressOnSegment: 0);

        var solution = _solver.Solve(cone, source, pathing, hitboxRadius: 65);

        Assert.NotNull(solution);
        Assert.Equal(0.5, solution.Value.Time, precision: 3);
    }

    [Fact]
    public void Solve_LinearVsPathingTarget_FindsInterceptionOnPath()
    {
        var skillshot = new Skillshot.Linear(Delay: 0.25f, Speed: 2000, Width: 60, Range: 1150);
        var source = new Point2D(0, 0);
        var pathing = new MovementState.Pathing(
            Waypoints: new[] { new Point2D(500, -200), new Point2D(500, 300) },
            Speed: 350,
            CurrentIndex: 1,
            ProgressOnSegment: 0);

        var solution = _solver.Solve(skillshot, source, pathing, hitboxRadius: 65);

        Assert.NotNull(solution);
        Assert.True(solution.Value.Time > 0.25, "Interception must happen after the cast delay");
        // Aim point must lie on the target's path line (x = 500), shifted by the cut
        Assert.Equal(500, solution.Value.Position.X, precision: 1);
    }

    [Fact]
    public void Solve_NonPathingTarget_ReturnsNull()
    {
        var skillshot = new Skillshot.Linear(Delay: 0.25f, Speed: 2000, Width: 60, Range: 1150);
        var walking = new MovementState.Walking(new Point2D(500, 0), new Vector2D(0, 350), null);

        var solution = _solver.Solve(skillshot, new Point2D(0, 0), walking, hitboxRadius: 65);

        Assert.Null(solution);
    }
}
