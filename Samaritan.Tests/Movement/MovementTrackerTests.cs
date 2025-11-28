namespace Samaritan.Tests.Movement;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Movement;

public class MovementTrackerTests
{
    [Fact]
    public void Update_FirstSample_SetsIdleState()
    {
        var tracker = new MovementTracker();

        tracker.Update(new Point2D(100, 100), gameTime: 0);

        Assert.IsType<MovementState.Idle>(tracker.CurrentState);
    }

    [Fact]
    public void Update_MultipleSamples_TransitionsToWalking()
    {
        var tracker = new MovementTracker();

        tracker.Update(new Point2D(0, 0), gameTime: 0);
        tracker.Update(new Point2D(100, 0), gameTime: 0.1);
        tracker.Update(new Point2D(200, 0), gameTime: 0.2);

        Assert.IsType<MovementState.Walking>(tracker.CurrentState);
    }

    [Fact]
    public void Update_StationaryTarget_RemainsIdle()
    {
        var tracker = new MovementTracker();

        tracker.Update(new Point2D(100, 100), gameTime: 0);
        tracker.Update(new Point2D(100, 100), gameTime: 0.1);
        tracker.Update(new Point2D(100, 100), gameTime: 0.2);

        Assert.IsType<MovementState.Idle>(tracker.CurrentState);
    }

    [Fact]
    public void NotifyDash_SetsCorrectState()
    {
        var tracker = new MovementTracker();
        tracker.Update(new Point2D(0, 0), gameTime: 0);

        tracker.NotifyDash(
            new Point2D(0, 0),
            new Point2D(500, 0),
            duration: 0.5,
            startTime: 0);

        Assert.IsType<MovementState.Dashing>(tracker.CurrentState);
    }

    [Fact]
    public void NotifyWaypoints_SetsWalkingState()
    {
        var tracker = new MovementTracker();
        tracker.Update(new Point2D(0, 0), gameTime: 0);

        tracker.NotifyWaypoints(new[] { new Point2D(500, 500) }, moveSpeed: 300);

        Assert.IsType<MovementState.Walking>(tracker.CurrentState);
    }

    [Fact]
    public void EstimateVelocity_NoSamples_ReturnsZero()
    {
        var tracker = new MovementTracker();

        var (velocity, confidence) = tracker.EstimateVelocity();

        Assert.Equal(0, velocity.X);
        Assert.Equal(0, velocity.Y);
        Assert.Equal(0, confidence);
    }

    [Fact]
    public void EstimateVelocity_WithSamples_CalculatesCorrectly()
    {
        var tracker = new MovementTracker();

        // Move 100 units in X over 0.1 seconds = 1000 u/s
        tracker.Update(new Point2D(0, 0), gameTime: 0);
        tracker.Update(new Point2D(100, 0), gameTime: 0.1);

        var (velocity, confidence) = tracker.EstimateVelocity();

        Assert.InRange(velocity.X, 950, 1050); // Allow some variance
        Assert.True(confidence > 0);
    }

    [Fact]
    public void DetectPathChange_StraightLine_ReturnsFalse()
    {
        var tracker = new MovementTracker();

        tracker.Update(new Point2D(0, 0), gameTime: 0);
        tracker.Update(new Point2D(100, 0), gameTime: 0.1);
        tracker.Update(new Point2D(200, 0), gameTime: 0.2);

        Assert.False(tracker.DetectPathChange());
    }

    [Fact]
    public void DetectPathChange_SharpTurn_ReturnsTrue()
    {
        var tracker = new MovementTracker();

        tracker.Update(new Point2D(0, 0), gameTime: 0);
        tracker.Update(new Point2D(100, 0), gameTime: 0.1);
        tracker.Update(new Point2D(100, 100), gameTime: 0.2); // 90 degree turn

        Assert.True(tracker.DetectPathChange());
    }

    [Fact]
    public void GetModel_ReturnsAppropriateModel()
    {
        var tracker = new MovementTracker();

        tracker.Update(new Point2D(0, 0), gameTime: 0);
        tracker.Update(new Point2D(100, 0), gameTime: 0.1);

        var model = tracker.GetModel();

        Assert.NotNull(model);
    }

    [Fact]
    public void GetModel_AfterDash_ReturnsAccelerationModel()
    {
        var tracker = new MovementTracker();
        tracker.Update(new Point2D(0, 0), gameTime: 0);

        tracker.NotifyDash(
            new Point2D(0, 0),
            new Point2D(500, 0),
            duration: 0.5,
            startTime: 0);

        var model = tracker.GetModel();

        Assert.IsType<AccelerationModel>(model);
    }

    [Fact]
    public void GetModel_AfterWaypoints_ReturnsWaypointModel()
    {
        var tracker = new MovementTracker();
        tracker.Update(new Point2D(0, 0), gameTime: 0);

        tracker.NotifyWaypoints(new[] { new Point2D(500, 500) }, moveSpeed: 300);

        var model = tracker.GetModel();

        Assert.IsType<WaypointModel>(model);
    }

    [Fact]
    public void SampleCount_TracksCorrectly()
    {
        var tracker = new MovementTracker(historyCapacity: 10);

        Assert.Equal(0, tracker.SampleCount);

        tracker.Update(new Point2D(0, 0), gameTime: 0);
        Assert.Equal(1, tracker.SampleCount);

        tracker.Update(new Point2D(100, 0), gameTime: 0.1);
        Assert.Equal(2, tracker.SampleCount);
    }

    [Fact]
    public void SampleCount_CapsAtCapacity()
    {
        var tracker = new MovementTracker(historyCapacity: 5);

        for (int i = 0; i < 10; i++)
        {
            tracker.Update(new Point2D(i * 100, 0), gameTime: i * 0.1);
        }

        Assert.Equal(5, tracker.SampleCount);
    }
}
