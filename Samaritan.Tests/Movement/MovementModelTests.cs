namespace Samaritan.Tests.Movement;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Movement;

public class ConstantVelocityModelTests
{
    [Fact]
    public void PredictPosition_ZeroDelta_ReturnsCurrentPosition()
    {
        var model = new ConstantVelocityModel(new Point2D(100, 200), new Vector2D(50, 50));

        var result = model.PredictPosition(0);

        Assert.Equal(100, result.X);
        Assert.Equal(200, result.Y);
    }

    [Fact]
    public void PredictPosition_PositiveDelta_MovesCorrectly()
    {
        var model = new ConstantVelocityModel(new Point2D(0, 0), new Vector2D(100, 50));

        var result = model.PredictPosition(2);

        Assert.Equal(200, result.X, precision: 1);
        Assert.Equal(100, result.Y, precision: 1);
    }

    [Fact]
    public void PredictVelocity_AlwaysReturnsConstant()
    {
        var velocity = new Vector2D(100, 50);
        var model = new ConstantVelocityModel(new Point2D(0, 0), velocity);

        var result1 = model.PredictVelocity(0);
        var result2 = model.PredictVelocity(5);

        Assert.Equal(velocity.X, result1.X);
        Assert.Equal(velocity.Y, result1.Y);
        Assert.Equal(velocity.X, result2.X);
        Assert.Equal(velocity.Y, result2.Y);
    }

    [Fact]
    public void GetConfidence_DecaysOverTime()
    {
        var model = new ConstantVelocityModel(new Point2D(0, 0), new Vector2D(100, 0));

        var conf0 = model.GetConfidence(0);
        var conf1 = model.GetConfidence(1);
        var conf2 = model.GetConfidence(2);

        Assert.True(conf0 > conf1);
        Assert.True(conf1 > conf2);
    }

    [Fact]
    public void Stationary_HasZeroVelocity()
    {
        var model = ConstantVelocityModel.Stationary(new Point2D(100, 200));

        var velocity = model.PredictVelocity(0);
        var position = model.PredictPosition(10);

        Assert.Equal(0, velocity.X);
        Assert.Equal(0, velocity.Y);
        Assert.Equal(100, position.X);
        Assert.Equal(200, position.Y);
    }
}

public class AccelerationModelTests
{
    [Fact]
    public void PredictPosition_AtStart_ReturnsStartPosition()
    {
        var model = new AccelerationModel(
            new Point2D(0, 0),
            new Point2D(500, 0),
            duration: 1.0,
            elapsedTime: 0);

        var result = model.PredictPosition(0);

        Assert.Equal(0, result.X, precision: 1);
    }

    [Fact]
    public void PredictPosition_AtEnd_ReturnsEndPosition()
    {
        var model = new AccelerationModel(
            new Point2D(0, 0),
            new Point2D(500, 0),
            duration: 1.0,
            elapsedTime: 0);

        var result = model.PredictPosition(1.0);

        Assert.Equal(500, result.X, precision: 1);
    }

    [Fact]
    public void PredictPosition_AfterEnd_ReturnsEndPosition()
    {
        var model = new AccelerationModel(
            new Point2D(0, 0),
            new Point2D(500, 0),
            duration: 1.0,
            elapsedTime: 0);

        var result = model.PredictPosition(5.0);

        Assert.Equal(500, result.X, precision: 1);
    }

    [Fact]
    public void PredictPosition_LinearEasing_MovesLinearly()
    {
        var model = new AccelerationModel(
            new Point2D(0, 0),
            new Point2D(1000, 0),
            duration: 1.0,
            elapsedTime: 0,
            easeType: EaseType.Linear);

        var resultHalf = model.PredictPosition(0.5);

        Assert.Equal(500, resultHalf.X, precision: 1);
    }

    [Fact]
    public void PredictPosition_EaseIn_SlowStart()
    {
        var model = new AccelerationModel(
            new Point2D(0, 0),
            new Point2D(1000, 0),
            duration: 1.0,
            elapsedTime: 0,
            easeType: EaseType.EaseIn);

        var resultHalf = model.PredictPosition(0.5);

        // EaseIn is t^2, so at t=0.5, position should be 0.25 * 1000 = 250
        Assert.Equal(250, resultHalf.X, precision: 10);
    }

    [Fact]
    public void GetConfidence_HighDuringDash()
    {
        var model = new AccelerationModel(
            new Point2D(0, 0),
            new Point2D(500, 0),
            duration: 1.0,
            elapsedTime: 0);

        var conf = model.GetConfidence(0.5);

        Assert.True(conf >= 0.9);
    }

    [Fact]
    public void GetConfidence_LowAfterDash()
    {
        var model = new AccelerationModel(
            new Point2D(0, 0),
            new Point2D(500, 0),
            duration: 0.5,
            elapsedTime: 0);

        var conf = model.GetConfidence(2.0);

        Assert.True(conf < 0.5);
    }

    [Fact]
    public void RemainingTime_CalculatesCorrectly()
    {
        var model = new AccelerationModel(
            new Point2D(0, 0),
            new Point2D(500, 0),
            duration: 1.0,
            elapsedTime: 0.3);

        Assert.Equal(0.7, model.RemainingTime, precision: 2);
    }
}

public class WaypointModelTests
{
    [Fact]
    public void PredictPosition_NoWaypoints_ReturnsCurrentPosition()
    {
        var model = new WaypointModel(
            new Point2D(100, 100),
            Array.Empty<Point2D>(),
            moveSpeed: 300);

        var result = model.PredictPosition(1.0);

        Assert.Equal(100, result.X);
        Assert.Equal(100, result.Y);
    }

    [Fact]
    public void PredictPosition_SingleWaypoint_MovesToward()
    {
        var model = new WaypointModel(
            new Point2D(0, 0),
            new[] { new Point2D(300, 0) },
            moveSpeed: 300);

        var result = model.PredictPosition(0.5);

        Assert.Equal(150, result.X, precision: 1);
    }

    [Fact]
    public void PredictPosition_ReachesWaypoint_Stops()
    {
        var model = new WaypointModel(
            new Point2D(0, 0),
            new[] { new Point2D(300, 0) },
            moveSpeed: 300);

        var result = model.PredictPosition(5.0); // Long past arrival

        Assert.Equal(300, result.X, precision: 1);
    }

    [Fact]
    public void PredictPosition_MultipleWaypoints_FollowsPath()
    {
        var model = new WaypointModel(
            new Point2D(0, 0),
            new[] { new Point2D(300, 0), new Point2D(300, 300) },
            moveSpeed: 300);

        // After 1.5 seconds: 1s to reach first waypoint, 0.5s toward second
        var result = model.PredictPosition(1.5);

        Assert.Equal(300, result.X, precision: 1);
        Assert.Equal(150, result.Y, precision: 1);
    }

    [Fact]
    public void PredictVelocity_MovingTowardWaypoint_ReturnsDirection()
    {
        var model = new WaypointModel(
            new Point2D(0, 0),
            new[] { new Point2D(300, 0) },
            moveSpeed: 300);

        var velocity = model.PredictVelocity(0);

        Assert.Equal(300, velocity.X, precision: 1);
        Assert.Equal(0, velocity.Y, precision: 1);
    }

    [Fact]
    public void PredictVelocity_AtDestination_ReturnsZero()
    {
        var model = new WaypointModel(
            new Point2D(0, 0),
            new[] { new Point2D(300, 0) },
            moveSpeed: 300);

        var velocity = model.PredictVelocity(5.0); // Past destination

        Assert.Equal(0, velocity.Length, precision: 1);
    }

    [Fact]
    public void GetConfidence_DecreasesWithTime()
    {
        var model = new WaypointModel(
            new Point2D(0, 0),
            new[] { new Point2D(600, 0) },
            moveSpeed: 300);

        var conf0 = model.GetConfidence(0);
        var conf1 = model.GetConfidence(1);

        Assert.True(conf0 > conf1);
    }
}
