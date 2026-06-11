namespace Samaritan.Tests.Engine;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Regression tests for cache key collisions: predictions made for one input
/// must never be served for a meaningfully different input.
/// </summary>
public class PredictionCachingTests
{
    private static readonly Skillshot.Linear EzrealQ = new(Delay: 0.25f, Speed: 2000, Width: 60, Range: 1150);

    [Fact]
    public void PredictFromState_DifferentHitboxRadius_DoesNotServeStaleCachedResult()
    {
        var caster = new Point2D(0, 0);
        var target = new MovementState.Walking(new Point2D(1080, 0), new Vector2D(0, 0), null);

        // Fresh engine: the ground-truth answer for hitbox 300
        var freshEngine = new PredictionEngine();
        var expected = freshEngine.PredictFromState(EzrealQ, caster, target, hitboxRadius: 300);

        // Primed engine: same state cached first with hitbox 5
        var primedEngine = new PredictionEngine();
        primedEngine.PredictFromState(EzrealQ, caster, target, hitboxRadius: 5);
        var actual = primedEngine.PredictFromState(EzrealQ, caster, target, hitboxRadius: 300);

        Assert.Equal(expected, actual);
    }

    [Fact]
    public void PredictFromState_PathingVsWalkingSamePositionAndVelocity_DoesNotServeStaleCachedResult()
    {
        var caster = new Point2D(0, 0);

        // Pathing target at (500, 0) moving up, but about to turn sharply
        var pathing = new MovementState.Pathing(
            Waypoints: new[] { new Point2D(500, 0), new Point2D(500, 10), new Point2D(0, 300) },
            Speed: 350,
            CurrentIndex: 1,
            ProgressOnSegment: 0);

        // Walking target with identical current position and velocity (no turn)
        var walking = new MovementState.Walking(new Point2D(500, 0), new Vector2D(0, 350), null);

        var freshEngine = new PredictionEngine();
        var expected = freshEngine.PredictFromState(EzrealQ, caster, walking, hitboxRadius: 65);

        var primedEngine = new PredictionEngine();
        primedEngine.PredictFromState(EzrealQ, caster, pathing, hitboxRadius: 65);
        var actual = primedEngine.PredictFromState(EzrealQ, caster, walking, hitboxRadius: 65);

        Assert.Equal(expected, actual);
    }

    [Fact]
    public void PredictFromState_CachedRepeatOfIdenticalQuery_ReturnsSameResult()
    {
        var caster = new Point2D(0, 0);
        var target = new MovementState.Walking(new Point2D(600, 0), new Vector2D(0, 350), null);

        var engine = new PredictionEngine();
        var first = engine.PredictFromState(EzrealQ, caster, target, hitboxRadius: 65);
        var second = engine.PredictFromState(EzrealQ, caster, target, hitboxRadius: 65);

        Assert.IsType<PredictionResult.Hit>(first);
        Assert.Equal(first, second);
    }
}
