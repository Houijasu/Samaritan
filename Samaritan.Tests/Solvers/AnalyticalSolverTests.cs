namespace Samaritan.Tests.Solvers;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Configuration;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Solvers;

public class AnalyticalSolverTests
{
    private readonly AnalyticalSolver _solver = new();

    [Fact]
    public void SolveLinear_StationaryTarget_ReturnsDirectHit()
    {
        // Arrange
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var source = new Point2D(0, 0);
        var target = new MovementState.Idle(new Point2D(500, 0));
        var hitboxRadius = 50.0;

        // Act
        var result = _solver.Solve(skillshot, source, target, hitboxRadius);

        // Assert
        Assert.NotNull(result);
        Assert.True(result.Value.Time > 0);
        Assert.True(result.Value.Time < 1); // Should hit within 1 second
        Assert.Equal(500, result.Value.Position.X, precision: 1);
    }

    [Fact]
    public void SolveLinear_MovingTarget_LeadsTarget()
    {
        // Arrange
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1500);
        var source = new Point2D(0, 0);
        // Target moving perpendicular to caster
        var target = new MovementState.Walking(
            Position: new Point2D(500, 0),
            Velocity: new Vector2D(0, 300),
            Destination: null);
        var hitboxRadius = 50.0;

        // Act
        var result = _solver.Solve(skillshot, source, target, hitboxRadius);

        // Assert
        Assert.NotNull(result);
        // Target should have moved in Y direction by the time projectile arrives
        Assert.True(result.Value.Position.Y > 0, "Should lead the target");
    }

    [Fact]
    public void SolveLinear_TargetOutOfRange_ReturnsNull()
    {
        // Arrange
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 500);
        var source = new Point2D(0, 0);
        var target = new MovementState.Idle(new Point2D(1000, 0)); // Out of range
        var hitboxRadius = 50.0;

        // Act
        var result = _solver.Solve(skillshot, source, target, hitboxRadius);

        // Assert
        Assert.Null(result);
    }

    [Fact]
    public void SolveLinear_WithDelay_AccountsForDelay()
    {
        // Arrange
        var delay = 0.25f;
        var skillshot = new Skillshot.Linear(Delay: delay, Speed: 1000, Width: 100, Range: 1000);
        var source = new Point2D(0, 0);
        var target = new MovementState.Walking(
            Position: new Point2D(500, 0),
            Velocity: new Vector2D(200, 0), // Moving away
            Destination: null);
        var hitboxRadius = 50.0;

        // Act
        var result = _solver.Solve(skillshot, source, target, hitboxRadius);

        // Assert
        Assert.NotNull(result);
        Assert.True(result.Value.Time >= delay, "Hit time should be >= delay");
        // Target moved during delay + travel time
        Assert.True(result.Value.Position.X > 500);
    }

    [Fact]
    public void SolveCircular_StationaryTarget_ReturnsHit()
    {
        // Arrange
        var skillshot = new Skillshot.Circular(Delay: 0.5f, Speed: 1000, Radius: 200, Range: 1000);
        var source = new Point2D(0, 0);
        var target = new MovementState.Idle(new Point2D(600, 0));
        var hitboxRadius = 50.0;

        // Act
        var result = _solver.Solve(skillshot, source, target, hitboxRadius);

        // Assert
        Assert.NotNull(result);
    }

    [Fact]
    public void SolveCone_TargetInRange_ReturnsHit()
    {
        // Arrange
        var skillshot = new Skillshot.Cone(Delay: 0.25f, Angle: 60, Range: 500);
        var source = new Point2D(0, 0);
        var target = new MovementState.Idle(new Point2D(300, 0));
        var hitboxRadius = 50.0;

        // Act
        var result = _solver.Solve(skillshot, source, target, hitboxRadius);

        // Assert
        Assert.NotNull(result);
        Assert.Equal(0.25, result.Value.Time, precision: 2);
    }

    [Fact]
    public void SolveCone_TargetOutOfRange_ReturnsNull()
    {
        // Arrange
        var skillshot = new Skillshot.Cone(Delay: 0.25f, Angle: 60, Range: 300);
        var source = new Point2D(0, 0);
        var target = new MovementState.Idle(new Point2D(500, 0));
        var hitboxRadius = 50.0;

        // Act
        var result = _solver.Solve(skillshot, source, target, hitboxRadius);

        // Assert
        Assert.Null(result);
    }

    [Fact]
    public void CanSolve_IdleTarget_ReturnsTrue()
    {
        var skillshot = new Skillshot.Linear(0, 1000, 100, 1000);
        var target = new MovementState.Idle(new Point2D(0, 0));

        Assert.True(_solver.CanSolve(skillshot, target));
    }

    [Fact]
    public void CanSolve_WalkingTarget_ReturnsTrue()
    {
        var skillshot = new Skillshot.Linear(0, 1000, 100, 1000);
        var target = new MovementState.Walking(new Point2D(0, 0), new Vector2D(100, 0), null);

        Assert.True(_solver.CanSolve(skillshot, target));
    }

    [Fact]
    public void CanSolve_DashingTarget_ReturnsFalse()
    {
        var skillshot = new Skillshot.Linear(0, 1000, 100, 1000);
        var target = new MovementState.Dashing(
            new Point2D(0, 0), new Point2D(500, 0), 0.5, 0);

        Assert.False(_solver.CanSolve(skillshot, target));
    }

    [Fact]
    public void SolveLinear_TargetMovingAway_StillHits()
    {
        // Arrange: Target moving away but slower than projectile
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 2000, Width: 100, Range: 2000);
        var source = new Point2D(0, 0);
        var target = new MovementState.Walking(
            Position: new Point2D(500, 0),
            Velocity: new Vector2D(300, 0), // Moving away at 300 u/s
            Destination: null);
        var hitboxRadius = 50.0;

        // Act
        var result = _solver.Solve(skillshot, source, target, hitboxRadius);

        // Assert
        Assert.NotNull(result);
        Assert.True(result.Value.Position.X > 500); // Hit position ahead of start
    }

    [Fact]
    public void SolveLinear_TargetMovingFasterThanProjectile_ReturnsNull()
    {
        // Arrange: Target moving away faster than projectile
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 500, Width: 100, Range: 2000);
        var source = new Point2D(0, 0);
        var target = new MovementState.Walking(
            Position: new Point2D(500, 0),
            Velocity: new Vector2D(600, 0), // Moving away faster than projectile
            Destination: null);
        var hitboxRadius = 50.0;

        // Act
        var result = _solver.Solve(skillshot, source, target, hitboxRadius);

        // Assert
        Assert.Null(result); // Can never catch up
    }
}
