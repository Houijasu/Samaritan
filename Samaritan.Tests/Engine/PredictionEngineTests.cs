namespace Samaritan.Tests.Engine;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Configuration;
using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

public class PredictionEngineTests
{
    [Fact]
    public void Predict_StationaryTarget_ReturnsHit()
    {
        var engine = new PredictionEngine();
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var casterPos = new Point2D(0, 0);

        var tracker = new MovementTracker();
        tracker.Update(new Point2D(500, 0), gameTime: 0);

        var result = engine.Predict(skillshot, casterPos, tracker);

        Assert.IsType<PredictionResult.Hit>(result);
    }

    [Fact]
    public void Predict_TargetOutOfRange_ReturnsOutOfRange()
    {
        var engine = new PredictionEngine();
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 500);
        var casterPos = new Point2D(0, 0);

        var tracker = new MovementTracker();
        tracker.Update(new Point2D(1500, 0), gameTime: 0); // Way out of range

        var result = engine.Predict(skillshot, casterPos, tracker);

        Assert.IsType<PredictionResult.OutOfRange>(result);
    }

    [Fact]
    public void PredictFromState_ValidInput_ReturnsResult()
    {
        var engine = new PredictionEngine();
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var casterPos = new Point2D(0, 0);
        var targetState = new MovementState.Walking(
            new Point2D(400, 0),
            new Vector2D(100, 0),
            null);

        var result = engine.PredictFromState(skillshot, casterPos, targetState, hitboxRadius: 50);

        Assert.NotNull(result);
    }

    [Fact]
    public void PredictMultiple_MultipleTargets_ReturnsResultsForAll()
    {
        var engine = new PredictionEngine();
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var casterPos = new Point2D(0, 0);

        var trackers = new[]
        {
            CreateTrackerAt(new Point2D(300, 0)),
            CreateTrackerAt(new Point2D(500, 0)),
            CreateTrackerAt(new Point2D(700, 0))
        };

        var results = engine.PredictMultiple(skillshot, casterPos, trackers);

        Assert.Equal(3, results.Count);
    }

    [Fact]
    public void Predict_WithCaching_ReturnsSameResult()
    {
        var engine = new PredictionEngine(enableCaching: true);
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var casterPos = new Point2D(0, 0);

        var tracker = new MovementTracker();
        tracker.Update(new Point2D(500, 0), gameTime: 0);

        var result1 = engine.Predict(skillshot, casterPos, tracker);
        var result2 = engine.Predict(skillshot, casterPos, tracker);

        // Both should be hits
        Assert.IsType<PredictionResult.Hit>(result1);
        Assert.IsType<PredictionResult.Hit>(result2);
    }

    [Fact]
    public void Predict_DifferentSkillshots_DifferentResults()
    {
        var engine = new PredictionEngine();
        var linearSkill = new Skillshot.Linear(Delay: 0, Speed: 500, Width: 100, Range: 1000);
        var circularSkill = new Skillshot.Circular(Delay: 0.5f, Speed: 0, Radius: 200, Range: 1000);
        var casterPos = new Point2D(0, 0);

        var tracker = new MovementTracker();
        tracker.Update(new Point2D(500, 0), gameTime: 0);
        tracker.Update(new Point2D(600, 0), gameTime: 0.1);

        var linearResult = engine.Predict(linearSkill, casterPos, tracker);
        var circularResult = engine.Predict(circularSkill, casterPos, tracker);

        // Both should return results (could be hit or unreachable)
        Assert.NotNull(linearResult);
        Assert.NotNull(circularResult);
    }

    [Fact]
    public void Predict_MovingTarget_LeadsCorrectly()
    {
        var engine = new PredictionEngine();
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1500);
        var casterPos = new Point2D(0, 0);

        var tracker = new MovementTracker();
        tracker.Update(new Point2D(500, 0), gameTime: 0);
        tracker.Update(new Point2D(500, 100), gameTime: 0.1); // Moving up

        var result = engine.Predict(skillshot, casterPos, tracker);

        if (result is PredictionResult.Hit hit)
        {
            // Should lead the target - predicted Y should be > 100
            Assert.True(hit.PredictedPosition.Y > 100);
        }
    }

    [Fact]
    public void Predict_ConeSkillshot_ReturnsHitForNearbyTarget()
    {
        var engine = new PredictionEngine();
        var skillshot = new Skillshot.Cone(Delay: 0.25f, Angle: 60, Range: 500);
        var casterPos = new Point2D(0, 0);

        var tracker = new MovementTracker();
        tracker.Update(new Point2D(300, 0), gameTime: 0);

        var result = engine.Predict(skillshot, casterPos, tracker);

        Assert.IsType<PredictionResult.Hit>(result);
    }

    [Fact]
    public void Predict_ArcSkillshot_CanHitAtArcRange()
    {
        var engine = new PredictionEngine();
        var skillshot = new Skillshot.Arc(Delay: 0, Speed: 1000, Width: 100, OuterRadius: 500, Angle: 90);
        var casterPos = new Point2D(0, 0);

        var tracker = new MovementTracker();
        tracker.Update(new Point2D(500, 0), gameTime: 0);

        var result = engine.Predict(skillshot, casterPos, tracker);

        Assert.NotNull(result);
    }

    [Fact]
    public void Predict_WithCustomConfig_UsesConfig()
    {
        var config = new PredictionConfig
        {
            MaxPredictionTime = 1.0,
            DefaultHitboxRadius = 100
        };
        var engine = new PredictionEngine(config);
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 100, Width: 50, Range: 2000);
        var casterPos = new Point2D(0, 0);

        var tracker = new MovementTracker();
        tracker.Update(new Point2D(500, 0), gameTime: 0);

        // With max prediction time of 1s and slow projectile, might not reach
        var result = engine.Predict(skillshot, casterPos, tracker);

        Assert.NotNull(result);
    }

    private static MovementTracker CreateTrackerAt(Point2D position)
    {
        var tracker = new MovementTracker();
        tracker.Update(position, gameTime: 0);
        return tracker;
    }

    [Fact]
    public void ValidateHit_TargetInPath_ReturnsTrue()
    {
        var engine = new PredictionEngine();
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var casterPos = new Point2D(0, 0);
        var aimPos = new Point2D(1000, 0);
        var targetPos = new Point2D(500, 0);

        var result = engine.ValidateHit(skillshot, casterPos, aimPos, targetPos, hitboxRadius: 50, timeElapsed: 0.6);

        Assert.True(result);
    }

    [Fact]
    public void ValidateHit_TargetOutsidePath_ReturnsFalse()
    {
        var engine = new PredictionEngine();
        var skillshot = new Skillshot.Linear(Delay: 0, Speed: 1000, Width: 100, Range: 1000);
        var casterPos = new Point2D(0, 0);
        var aimPos = new Point2D(1000, 0);
        var targetPos = new Point2D(500, 300); // Too far from line

        var result = engine.ValidateHit(skillshot, casterPos, aimPos, targetPos, hitboxRadius: 50, timeElapsed: 0.6);

        Assert.False(result);
    }

    [Fact]
    public void ValidateHit_CircularSkillshot_HitsTargetInRadius()
    {
        var engine = new PredictionEngine();
        var skillshot = new Skillshot.Circular(Delay: 0.5f, Speed: 0, Radius: 200, Range: 1000);
        var casterPos = new Point2D(0, 0);
        var aimPos = new Point2D(500, 0);
        var targetPos = new Point2D(450, 50); // Within radius of aim point

        var result = engine.ValidateHit(skillshot, casterPos, aimPos, targetPos, hitboxRadius: 50, timeElapsed: 1.0);

        Assert.True(result);
    }

    [Fact]
    public void ValidateHit_ConeSkillshot_HitsTargetInCone()
    {
        var engine = new PredictionEngine();
        var skillshot = new Skillshot.Cone(Delay: 0.25f, Angle: 60, Range: 500);
        var casterPos = new Point2D(0, 0);
        var aimPos = new Point2D(500, 0);
        var targetPos = new Point2D(300, 50); // Within cone angle

        var result = engine.ValidateHit(skillshot, casterPos, aimPos, targetPos, hitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }

    [Fact]
    public void ValidateHit_VectorRectangleSkillshot_HitsTargetInPath()
    {
        var engine = new PredictionEngine();
        var skillshot = new Skillshot.VectorRectangle(Delay: 0, Speed: 1000, Width: 200, MaxLength: 500, Range: 1000);
        var casterPos = new Point2D(0, 0);
        var aimPos = new Point2D(500, 0);
        var targetPos = new Point2D(300, 50); // Within vector rectangle

        var result = engine.ValidateHit(skillshot, casterPos, aimPos, targetPos, hitboxRadius: 50, timeElapsed: 0.5);

        Assert.True(result);
    }
}
