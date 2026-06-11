namespace Samaritan.Tests.Collision;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Collision;

public class LinearCollisionDetectorTests
{
    private readonly LinearCollisionDetector _detector = new();

    [Fact]
    public void WillHit_TargetAtProjectileHead_ReturnsTrue()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(1000, 0);
        // After 0.5s the head is at x=500 - exactly on the target
        var target = new Point2D(500, 0);

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_TargetOutsidePath_ReturnsFalse()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(1000, 0);
        var target = new Point2D(500, 200); // Too far from line

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_BeforeDelay_ReturnsFalse()
    {
        var skillshot = new Skillshot.Linear(Delay: 0.5f, Speed: 1000, Width: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(1000, 0);
        var target = new Point2D(100, 0);

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.2);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetBeyondProjectile_ReturnsFalse()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(1000, 0);
        var target = new Point2D(800, 0);

        // Projectile only traveled 500 units
        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetFarBehindProjectileHead_ReturnsFalse()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(1000, 0);
        // After 0.8s the head is at x=800; a target at x=100 sits in the wake
        // the projectile passed long ago and must not register as a hit now.
        var target = new Point2D(100, 0);

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.8);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_AfterProjectileExpired_ReturnsFalse()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(1000, 0);
        // Missile reached max range (1000) at t=1.0; well after that, a target
        // standing at the end of the path is no longer hit.
        var target = new Point2D(1000, 0);

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 2.0);

        Assert.False(result);
    }

    [Fact]
    public void SweptContact_CrossesRadiusMidStep_ReturnsEntryFraction()
    {
        // Relative offset sweeps from (200,10) to (-200,10): dips to 10 units
        // from the origin, entering the 100-radius at x = +99.5
        var hit = LinearCollisionDetector.SweptContact(
            new Vector2D(200, 10), new Vector2D(-200, 10), radius: 100, out var fraction);

        Assert.True(hit);
        Assert.Equal(0.251, fraction, precision: 2);
    }

    [Fact]
    public void SweptContact_StartsInsideRadius_ReturnsZeroFraction()
    {
        var hit = LinearCollisionDetector.SweptContact(
            new Vector2D(50, 0), new Vector2D(300, 0), radius: 100, out var fraction);

        Assert.True(hit);
        Assert.Equal(0, fraction);
    }

    [Fact]
    public void SweptContact_StaysOutsideRadius_ReturnsFalse()
    {
        var hit = LinearCollisionDetector.SweptContact(
            new Vector2D(200, 150), new Vector2D(-200, 150), radius: 100, out _);

        Assert.False(hit);
    }

    [Fact]
    public void SweptContact_NoRelativeMotionOutside_ReturnsFalse()
    {
        var hit = LinearCollisionDetector.SweptContact(
            new Vector2D(200, 0), new Vector2D(200, 0), radius: 100, out _);

        Assert.False(hit);
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
        var aim = new Point2D(500, 0);
        var target = new Point2D(500, 50); // Within radius of the aim point

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 1.0);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_TargetOutsideCircle_ReturnsFalse()
    {
        var skillshot = new Skillshot.Circular(Delay: 0.5f, Speed: 0, Radius: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(500, 0);
        var target = new Point2D(500, 300); // Outside radius

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 1.0);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_BeforeTravelComplete_ReturnsFalse()
    {
        var skillshot = new Skillshot.Circular(Delay: 0, Speed: 1000, Radius: 200, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(500, 0);
        var target = new Point2D(500, 0);

        // Not enough time for the projectile to reach the aim point (needs 0.5s)
        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.3);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_ImpactIsAtAimPointNotAtTarget_ReturnsFalse()
    {
        var skillshot = new Skillshot.Circular(Delay: 0.5f, Speed: 0, Radius: 100, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(300, 0);
        // Target on the aim ray but far past the impact point; the old detector
        // wrongly slid the impact out to the target's own distance.
        var target = new Point2D(700, 0);

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 1.0);

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
        var aim = new Point2D(500, 0);
        var target = new Point2D(300, 50); // Within cone angle

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_TargetOutsideCone_ReturnsFalse()
    {
        var skillshot = new Skillshot.Cone(Delay: 0.25f, Angle: 30, Range: 500);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(500, 0);
        var target = new Point2D(300, 300); // Outside cone angle

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetBeyondRange_ReturnsFalse()
    {
        var skillshot = new Skillshot.Cone(Delay: 0, Angle: 60, Range: 300);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(300, 0);
        var target = new Point2D(500, 0); // Beyond range

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetAtOrigin_ReturnsTrue()
    {
        var skillshot = new Skillshot.Cone(Delay: 0, Angle: 60, Range: 500);
        var origin = new Point2D(100, 100);
        var aim = new Point2D(600, 100);
        var target = new Point2D(100, 100); // At origin

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.5);

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
        var aim = new Point2D(500, 0);
        var target = new Point2D(500, 0); // On outer arc edge

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 1.0);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_TargetInsideArc_ReturnsFalse()
    {
        var skillshot = new Skillshot.Arc(Delay: 0, Speed: 1000, Width: 50, OuterRadius: 500, Angle: 90);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(500, 0);
        var target = new Point2D(200, 0); // Too close to origin

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 30, timeElapsed: 1.0);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_BeforeDelay_ReturnsFalse()
    {
        var skillshot = new Skillshot.Arc(Delay: 0.5f, Speed: 1000, Width: 100, OuterRadius: 500, Angle: 90);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(500, 0);
        var target = new Point2D(500, 0);

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.2);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_CounterClockwiseArc_HitsAboveAxis()
    {
        // Counter-clockwise arc starting right, should curve upward (positive Y)
        var skillshot = new Skillshot.Arc(Delay: 0, Speed: 1000, Width: 100, OuterRadius: 400, Angle: 90, Clockwise: false);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(400, 0);
        var targetAbove = new Point2D(283, 283); // ~45 degrees, on arc path CCW

        var result = _detector.WillHit(skillshot, origin, aim, targetAbove, targetHitboxRadius: 50, timeElapsed: 1.0);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_ClockwiseArc_HitsBelowAxis()
    {
        // Clockwise arc starting right, should curve downward (negative Y)
        var skillshot = new Skillshot.Arc(Delay: 0, Speed: 1000, Width: 100, OuterRadius: 400, Angle: 90, Clockwise: true);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(400, 0);
        var targetBelow = new Point2D(283, -283); // ~-45 degrees, on arc path CW

        var result = _detector.WillHit(skillshot, origin, aim, targetBelow, targetHitboxRadius: 50, timeElapsed: 1.0);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_ClockwiseArc_MissesAboveAxis()
    {
        // Clockwise arc should NOT hit targets above axis
        var skillshot = new Skillshot.Arc(Delay: 0, Speed: 1000, Width: 100, OuterRadius: 400, Angle: 90, Clockwise: true);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(400, 0);
        var targetAbove = new Point2D(283, 283); // Above axis - wrong side for CW

        var result = _detector.WillHit(skillshot, origin, aim, targetAbove, targetHitboxRadius: 30, timeElapsed: 1.0);

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
        var aim = new Point2D(200, 0); // Rectangle centered at the aim point
        var target = new Point2D(200, 50);

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_TargetOutsideWidth_ReturnsFalse()
    {
        var skillshot = new Skillshot.Rectangle(Delay: 0, Speed: 1000, Width: 100, Length: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(200, 0);
        var target = new Point2D(200, 200); // Outside width

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 30, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetBeyondLength_ReturnsFalse()
    {
        var skillshot = new Skillshot.Rectangle(Delay: 0, Speed: 1000, Width: 200, Length: 300, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(100, 0);
        var target = new Point2D(400, 0); // Beyond the rectangle placed at x=100

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 30, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_BeforeTravelToLandingPoint_ReturnsFalse()
    {
        var skillshot = new Skillshot.Rectangle(Delay: 0, Speed: 1000, Width: 200, Length: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(800, 0);
        var target = new Point2D(800, 0);

        // Needs 0.8s of travel before the effect lands at the aim point
        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_DiagonalDirection_WorksCorrectly()
    {
        var skillshot = new Skillshot.Rectangle(Delay: 0, Speed: 1000, Width: 200, Length: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(200, 200);
        var target = new Point2D(200, 200); // At the landing point

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }
}

public class VectorRectangleCollisionDetectorTests
{
    private readonly VectorRectangleCollisionDetector _detector = new();

    [Fact]
    public void WillHit_FrontCrossingTarget_ReturnsTrue()
    {
        // Beam starts at the aim point (100,0) and sweeps along +X at 1000 u/s
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(100, 0);
        var target = new Point2D(300, 50);

        // At t=0.2 the front is 200 units past the start - exactly on the target
        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.2);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_FrontAlreadyPastTarget_ReturnsFalse()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(100, 0);
        var target = new Point2D(300, 50);

        // The front crossed the target around t=0.2; at t=0.45 it is long past
        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.45);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetOutsideWidth_ReturnsFalse()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 100, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(100, 0);
        var target = new Point2D(300, 200); // Outside width

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 30, timeElapsed: 0.2);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetBeyondCurrentTravel_ReturnsFalse()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 800, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(50, 0);
        var target = new Point2D(600, 0); // 550 units past the start; front is only at 500

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 30, timeElapsed: 0.5);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetAtMaxLength_ReturnsTrue()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(10, 0);
        var target = new Point2D(500, 0); // 490 units past the start

        // At t=0.5 the front arrives at MaxLength (500) and is crossing the target
        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }

    [Fact]
    public void WillHit_BeforeDelay_ReturnsFalse()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0.5f, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(100, 0);
        var target = new Point2D(100, 0);

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.2);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_TargetBehindBeamStart_ReturnsFalse()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(100, 0);
        var target = new Point2D(-200, 0); // Behind the beam start

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 30, timeElapsed: 0.2);

        Assert.False(result);
    }

    [Fact]
    public void WillHit_DiagonalDirection_WorksCorrectly()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);
        var origin = new Point2D(0, 0);
        var aim = new Point2D(100, 100);
        var target = new Point2D(200, 200); // ~141 units past the start, along the diagonal

        var result = _detector.WillHit(skillshot, origin, aim, target, targetHitboxRadius: 50, timeElapsed: 0.2);

        Assert.True(result);
    }
}
