namespace Samaritan.Tests.Engine;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Configuration;
using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// The legacy engine is the pre-audit algorithm preserved verbatim for A/B
/// comparison in the simulation. These tests pin the expected differences
/// between BEFORE (legacy) and AFTER (current) - they document the legacy
/// behavior including its known defects, which must stay reproducible.
/// </summary>
public class LegacyPredictionEngineTests
{
    [Fact]
    public void PredictFromState_LinearVsWalkingPerpendicular_ReturnsHit()
    {
        var legacy = new LegacyPredictionEngine();
        var skillshot = new Skillshot.Linear(Delay: 0.25f, Speed: 2000, Width: 60, Range: 1150);
        var target = new MovementState.Walking(new Point2D(600, -200), new Vector2D(0, 350), null);

        var result = legacy.PredictFromState(skillshot, new Point2D(0, 0), target, hitboxRadius: 65);

        Assert.IsType<PredictionResult.Hit>(result);
    }

    [Fact]
    public void PredictFromState_CircularVsPathing_LegacyMissesWhereCurrentHits()
    {
        var luxE = new Skillshot.Circular(Delay: 0.25f, Speed: 1200, Radius: 350, Range: 1100);
        var caster = new Point2D(0, 0);
        var pathing = new MovementState.Pathing(
            Waypoints: new[] { new Point2D(400, -300), new Point2D(400, 100), new Point2D(800, 100) },
            Speed: 350,
            CurrentIndex: 1,
            ProgressOnSegment: 0);

        // Legacy bug (preserved by design): instant-speed sentinel poisons the
        // waypoint quadratic with NaN, so circular vs pathing never hits
        var legacyResult = new LegacyPredictionEngine().PredictFromState(luxE, caster, pathing, 65);
        Assert.IsType<PredictionResult.Unreachable>(legacyResult);

        var currentResult = new PredictionEngine(enableCaching: false).PredictFromState(luxE, caster, pathing, 65);
        Assert.IsType<PredictionResult.Hit>(currentResult);
    }

    [Fact]
    public void PredictFromState_NearFormerCorrectionPole_LegacyFailsWhereCurrentHits()
    {
        // hitboxRatio = 0.5 with cosTheta just past 0.75 - the legacy correction
        // divisor crosses zero here and reports a hittable target as unreachable
        var config = new PredictionConfig { ServerTickRateHz = double.PositiveInfinity };
        var skillshot = new Skillshot.Linear(Delay: 0.25f, Speed: 2000, Width: 60, Range: 2000);
        var caster = new Point2D(0, 0);

        const double CosTheta = 0.7501;
        var sinTheta = Math.Sqrt(1 - CosTheta * CosTheta);
        var target = new MovementState.Walking(
            new Point2D(500, 0),
            new Vector2D(380 * CosTheta, 380 * sinTheta),
            null);

        var legacyResult = new LegacyPredictionEngine(config).PredictFromState(skillshot, caster, target, 65);
        Assert.IsType<PredictionResult.Unreachable>(legacyResult);

        var currentResult = new PredictionEngine(config, enableCaching: false).PredictFromState(skillshot, caster, target, 65);
        Assert.IsType<PredictionResult.Hit>(currentResult);
    }
}
