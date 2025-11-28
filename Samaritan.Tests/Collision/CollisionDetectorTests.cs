namespace Samaritan.Tests.Collision;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Collision;

public class LinearCollisionDetectorTests
{
    private readonly LinearCollisionDetector _detector = new();

    [Fact]
    public void WillHit_TargetInPath_ReturnsTrue()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(500, 0);

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.6);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_TargetOutsidePath_ReturnsFalse()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(500, 200); // Too far from line

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.6);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_BeforeDelay_ReturnsFalse()
    {
        var skillshot = new Skillshot.Linear(Delay: 0.5f, Speed: 1000, Width: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(100, 0);

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.2);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetBeyondProjectile_ReturnsFalse()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(800, 0);

        // Projectile only traveled 500 units
        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void PointToSegmentDistance_PointOnSegment_ReturnsZero()
    {
        var result = LinearCollisionDetector.PointToSegmentDistance(
            new Point2D(50, 0),
            new Point2D(0, 0),
            new Point2D(100, 0));

        Assert.Equal(0, result, 1);
    }

    [Fact]
    public void PointToSegmentDistance_PointOffSegment_ReturnsDistance()
    {
        var result = LinearCollisionDetector.PointToSegmentDistance(
            new Point2D(50, 30),
            new Point2D(0, 0),
            new Point2D(100, 0));

        Assert.Equal(30, result, 1);
    }
}

public class CircularCollisionDetectorTests
{
    private readonly CircularCollisionDetector _detector = new();

    [Fact]
    public void WillHit_TargetInCircle_ReturnsTrue()
    {
        var skillshot = new Skillshot.Circular(Delay: 0.5f, Speed: 0, Radius: 200, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(500, 50); // Within radius

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 1.0);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_TargetOutsideCircle_ReturnsFalse()
    {
        var skillshot = new Skillshot.Circular(Delay: 0.5f, Speed: 0, Radius: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(500, 300); // Outside radius

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 1.0);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_BeforeTravelComplete_ReturnsFalse()
    {
        var skillshot = new Skillshot.Circular(Delay: 0, Speed: 1000, Radius: 200, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(500, 0);

        // Not enough time to reach target
        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.3);

        Assert.False(result);
    }
}

public class ConeCollisionDetectorTests
{
    private readonly ConeCollisionDetector _detector = new();

    [Fact]
    public void WillHit_TargetInCone_ReturnsTrue()
    {
        var skillshot = new Skillshot.Cone(Delay: 0.25f, Angle: 60, Range: 500);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(300, 50); // Within cone angle

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_TargetOutsideCone_ReturnsFalse()
    {
        var skillshot = new Skillshot.Cone(Delay: 0.25f, Angle: 30, Range: 500);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(300, 300); // Outside cone angle

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetBeyondRange_ReturnsFalse()
    {
        var skillshot = new Skillshot.Cone(Delay: 0, Angle: 60, Range: 300);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(500, 0); // Beyond range

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetAtOrigin_ReturnsTrue()
    {
        var skillshot = new Skillshot.Cone(Delay: 0, Angle: 60, Range: 500);
        var origin = new Point2D(100, 100);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(100, 100); // At origin

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }
}

public class ArcCollisionDetectorTests
{
    private readonly ArcCollisionDetector _detector = new();

    [Fact]
    public void WillHit_TargetOnArc_ReturnsTrue()
    {
        var skillshot = new Skillshot.Arc(Delay: 0, Speed: 1000, Width: 100, OuterRadius: 500, Angle: 90);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(500, 0); // On outer arc edge

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 1.0);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_TargetInsideArc_ReturnsFalse()
    {
        var skillshot = new Skillshot.Arc(Delay: 0, Speed: 1000, Width: 50, OuterRadius: 500, Angle: 90);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(200, 0); // Too close to origin

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 30, timeElapsed: 1.0);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_BeforeDelay_ReturnsFalse()
    {
        var skillshot = new Skillshot.Arc(Delay: 0.5f, Speed: 1000, Width: 100, OuterRadius: 500, Angle: 90);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(500, 0);

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.2);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_CounterClockwiseArc_HitsAboveAxis()
    {
        // Counter-clockwise arc starting right, should curve upward (positive Y)
        var skillshot = new Skillshot.Arc(Delay: 0, Speed: 1000, Width: 100, OuterRadius: 400, Angle: 90, Clockwise: false);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var targetAbove = new Point2D(283, 283); // ~45 degrees, on arc path CCW

        var result = _detector.WillHit(skillshot, origin, direction, targetAbove, targetHitboxRadius: 50, timeElapsed: 1.0);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_ClockwiseArc_HitsBelowAxis()
    {
        // Clockwise arc starting right, should curve downward (negative Y)
        var skillshot = new Skillshot.Arc(Delay: 0, Speed: 1000, Width: 100, OuterRadius: 400, Angle: 90, Clockwise: true);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var targetBelow = new Point2D(283, -283); // ~-45 degrees, on arc path CW

        var result = _detector.WillHit(skillshot, origin, direction, targetBelow, targetHitboxRadius: 50, timeElapsed: 1.0);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_ClockwiseArc_MissesAboveAxis()
    {
        // Clockwise arc should NOT hit targets above axis
        var skillshot = new Skillshot.Arc(Delay: 0, Speed: 1000, Width: 100, OuterRadius: 400, Angle: 90, Clockwise: true);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var targetAbove = new Point2D(283, 283); // Above axis - wrong side for CW

        var result = _detector.WillHit(skillshot, origin, direction, targetAbove, targetHitboxRadius: 30, timeElapsed: 1.0);

        Assert.False(result);
    }
}

public class RectangleCollisionDetectorTests
{
    private readonly RectangleCollisionDetector _detector = new();

    [Fact]
    public void WillHit_TargetInRectangle_ReturnsTrue()
    {
        var skillshot = new Skillshot.Rectangle(Delay: 0, Speed: 1000, Width: 200, Length: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(200, 50); // Within rectangle

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_TargetOutsideWidth_ReturnsFalse()
    {
        var skillshot = new Skillshot.Rectangle(Delay: 0, Speed: 1000, Width: 100, Length: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(200, 200); // Outside width

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 30, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetBeyondLength_ReturnsFalse()
    {
        var skillshot = new Skillshot.Rectangle(Delay: 0, Speed: 1000, Width: 200, Length: 300, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(400, 0); // Beyond length

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 30, timeElapsed: 0.3);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_DiagonalDirection_WorksCorrectly()
    {
        var skillshot = new Skillshot.Rectangle(Delay: 0, Speed: 1000, Width: 200, Length: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 1).Normalize();
        var target = new Point2D(200, 200); // On diagonal

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }
}

public class VectorRectangleCollisionDetectorTests
{
    private readonly VectorRectangleCollisionDetector _detector = new();

    [Fact]
    public void WillHit_TargetInVector_ReturnsTrue()
    {
        // Viktor E style - extends from origin toward target
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(300, 50); // Within vector rectangle

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_TargetOutsideWidth_ReturnsFalse()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 100, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(300, 200); // Outside width

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 30, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetBeyondCurrentTravel_ReturnsFalse()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 800, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(600, 0); // Beyond current travel (500 units at 0.5s)

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 30, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetAtMaxLength_ReturnsTrue()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(500, 0); // At max length

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 1.0);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_BeforeDelay_ReturnsFalse()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0.5f, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(100, 0);

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.2);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetBehindOrigin_ReturnsFalse()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var target = new Point2D(-200, 0); // Behind origin

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 30, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_DiagonalDirection_WorksCorrectly()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 1).Normalize();
        var target = new Point2D(200, 200); // On diagonal

        var result = _detector.WillHit(skillshot, origin, direction, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }
}
