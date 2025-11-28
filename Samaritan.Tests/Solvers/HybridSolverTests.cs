namespace Samaritan.Tests.Solvers;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Solvers;

public class HybridSolverTests
{
    private readonly HybridSolver _solver = new();

    [Fact]
    public void Solve_SimpleCase_UsesAnalytical()
    {
        // Arrange - simple case that analytical can handle
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var source = new Point2D(0, 0);
        var target = new MovementState.Idle(new Point2D(500, 0));
        var hitboxRadius = 50.0;

        // Act
        var result = _solver.Solve(skillshot, source, target, hitboxRadius);

        // Assert
        Assert.NotNull(result);
        Assert.True(result.Value.Confidence > 0.9); // Analytical gives high confidence
    }

    [Fact]
    public void Solve_DashingTarget_FallsBackToNumerical()
    {
        // Arrange - dashing requires numerical
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1500, Width: 100, Range: 1500);
        var source = new Point2D(0, 0);
        var target = new MovementState.Dashing(
            new Point2D(300, 0), new Point2D(800, 0), 0.5, 0);
        var hitboxRadius = 50.0;

        // Act
        var result = _solver.Solve(skillshot, source, target, hitboxRadius);

        // Assert
        Assert.NotNull(result);
    }

    [Fact]
    public void Solve_ArcSkillshot_UsesNumerical()
    {
        // Arrange - Arc always needs numerical
        var skillshot = new Skillshot.Arc(Delay: 0, Speed: 1000, Width: 100, OuterRadius: 500, Angle: 90);
        var source = new Point2D(0, 0);
        var target = new MovementState.Idle(new Point2D(500, 0));
        var hitboxRadius = 50.0;

        // Act
        var result = _solver.Solve(skillshot, source, target, hitboxRadius);

        // Assert
        Assert.NotNull(result);
    }

    [Fact]
    public void CanSolve_AlwaysReturnsTrue()
    {
        var skillshot = new Skillshot.Linear(0, 1000, 100, 1000);
        var target = new MovementState.Idle(new Point2D(0, 0));

        Assert.True(_solver.CanSolve(skillshot, target));
    }

    [Fact]
    public void Name_ReturnsHybrid()
    {
        Assert.Equal("Hybrid", _solver.Name);
    }
}
