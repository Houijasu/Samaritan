namespace Samaritan.Tests.Collision;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Collision;

public class CollisionValidationServiceTests
{
    private readonly CollisionValidationService _service = new();

    [Fact]
    public void GetDetector_LinearSkillshot_ReturnsLinearDetector()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);

        var detector = _service.GetDetector(skillshot);

        Assert.IsType<LinearCollisionDetector>(detector);
    }

    [Fact]
    public void GetDetector_CircularSkillshot_ReturnsCircularDetector()
    {
        var skillshot = new Skillshot.Circular(Delay: 0, Speed: 1000, Radius: 100, Range: 1000);

        var detector = _service.GetDetector(skillshot);

        Assert.IsType<CircularCollisionDetector>(detector);
    }

    [Fact]
    public void GetDetector_ConeSkillshot_ReturnsConeDetector()
    {
        var skillshot = new Skillshot.Cone(Delay: 0, Angle: 60, Range: 500);

        var detector = _service.GetDetector(skillshot);

        Assert.IsType<ConeCollisionDetector>(detector);
    }

    [Fact]
    public void GetDetector_ArcSkillshot_ReturnsArcDetector()
    {
        var skillshot = new Skillshot.Arc(Delay: 0, Speed: 1000, Width: 100, OuterRadius: 500, Angle: 90);

        var detector = _service.GetDetector(skillshot);

        Assert.IsType<ArcCollisionDetector>(detector);
    }

    [Fact]
    public void GetDetector_RectangleSkillshot_ReturnsRectangleDetector()
    {
        var skillshot = new Skillshot.Rectangle(Delay: 0, Speed: 1000, Width: 200, Length: 500, Range: 1000);

        var detector = _service.GetDetector(skillshot);

        Assert.IsType<RectangleCollisionDetector>(detector);
    }

    [Fact]
    public void GetDetector_VectorRectangleSkillshot_ReturnsVectorRectangleDetector()
    {
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);

        var detector = _service.GetDetector(skillshot);

        Assert.IsType<VectorRectangleCollisionDetector>(detector);
    }

    [Fact]
    public void ValidateHit_LinearSkillshotHits_ReturnsTrue()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var casterPos = new Point2D(0, 0);
        var aimPos = new Point2D(1000, 0);
        var targetPos = new Point2D(500, 0);

        var result = _service.ValidateHit(skillshot, casterPos, aimPos, targetPos, targetHitboxRadius: 50, timeElapsed: 0.6);

        Assert.True(result);
    }

    [Fact]
    public void ValidateHit_LinearSkillshotMisses_ReturnsFalse()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var casterPos = new Point2D(0, 0);
        var aimPos = new Point2D(1000, 0);
        var targetPos = new Point2D(500, 300); // Far from line

        var result = _service.ValidateHit(skillshot, casterPos, aimPos, targetPos, targetHitboxRadius: 50, timeElapsed: 0.6);

        Assert.False(result);
    }

    [Fact]
    public void ValidatePrediction_ValidPrediction_ReturnsTrue()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var casterPos = new Point2D(0, 0);
        var predictedPos = new Point2D(500, 0);

        var result = _service.ValidatePrediction(skillshot, casterPos, predictedPos, targetHitboxRadius: 50, interceptionTime: 0.5);

        Assert.True(result);
    }

    [Fact]
    public void SimulateCollision_StationaryTargetInPath_ReturnsHitTime()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var casterPos = new Point2D(0, 0);
        var aimDirection = new Vector2D(1, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 0);

        var hitTime = _service.SimulateCollision(
            skillshot, casterPos, aimDirection, targetPos, targetVel, targetHitboxRadius: 50);

        Assert.NotNull(hitTime);
        Assert.True(hitTime < 1.0); // Should hit within 1 second
    }

    [Fact]
    public void SimulateCollision_TargetMovingAway_ReturnsNull()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 500, Width: 50, Range: 500);
        var casterPos = new Point2D(0, 0);
        var aimDirection = new Vector2D(1, 0);
        var targetPos = new Point2D(400, 0);
        var targetVel = new Vector2D(600, 0); // Moving faster than projectile

        var hitTime = _service.SimulateCollision(
            skillshot, casterPos, aimDirection, targetPos, targetVel, targetHitboxRadius: 30, maxTime: 2.0);

        Assert.Null(hitTime);
    }

    [Fact]
    public void SimulateCollision_TargetMovingIntoPath_ReturnsHitTime()
    {
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var casterPos = new Point2D(0, 0);
        var aimDirection = new Vector2D(1, 0);
        var targetPos = new Point2D(500, 200); // Above the line
        var targetVel = new Vector2D(0, -400); // Moving down into path

        var hitTime = _service.SimulateCollision(
            skillshot, casterPos, aimDirection, targetPos, targetVel, targetHitboxRadius: 50, maxTime: 2.0);

        Assert.NotNull(hitTime);
    }
}
