namespace Samaritan.Tests.Engine;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Golden-value pins for the Gagong port: performance refactors must be
/// behavior-identical, so these assert the exact outputs (captured from the
/// straightforward port) to within numerical noise.
/// </summary>
public class GagongGoldenValueTests
{
    private const double Tolerance = 1e-6;

    private static readonly Skillshot.Linear NidaleeQ = new(
        Delay: 0.25f, Speed: 1300, Width: 40, Range: 1500);

    private static readonly Point2D Caster = new(0, 0);

    [Fact]
    public void WalkingPerpendicular_MatchesGoldenValues()
    {
        var target = new MovementState.Walking(new Point2D(600, -200), new Vector2D(0, 350), null);

        var hit = Assert.IsType<PredictionResult.Hit>(
            new GagongPredictionEngine().PredictFromState(NidaleeQ, Caster, target, 65));

        Assert.Equal(596.1942915943303, hit.CastPosition.X, Tolerance);
        Assert.Equal(-27.748156662483233, hit.CastPosition.Y, Tolerance);
        Assert.Equal(0.7257741063153054, hit.InterceptionTime, Tolerance);
        Assert.Equal(600, hit.PredictedPosition.X, Tolerance);
        Assert.Equal(54.020937210356884, hit.PredictedPosition.Y, Tolerance);
    }

    [Fact]
    public void CrossingTarget_MatchesGoldenValues()
    {
        var target = new MovementState.Walking(new Point2D(1000, 0), new Vector2D(-120, 330), null);

        var hit = Assert.IsType<PredictionResult.Hit>(
            new GagongPredictionEngine().PredictFromState(NidaleeQ, Caster, target, 65));

        Assert.Equal(902.7711208531069, hit.CastPosition.X, Tolerance);
        Assert.Equal(248.7239818528754, hit.CastPosition.Y, Tolerance);
        Assert.Equal(0.9869802554257301, hit.InterceptionTime, Tolerance);
    }

    [Fact]
    public void ZigzagPath_MatchesGoldenValues()
    {
        var pathing = new MovementState.Pathing(
            Waypoints: new[]
            {
                new Point2D(500, 0),
                new Point2D(600, 150),
                new Point2D(700, -100),
                new Point2D(800, 100)
            },
            Speed: 400,
            CurrentIndex: 1,
            ProgressOnSegment: 0);

        var hit = Assert.IsType<PredictionResult.Hit>(
            new GagongPredictionEngine().PredictFromState(NidaleeQ, Caster, pathing, 65));

        Assert.Equal(634.0686671590557, hit.CastPosition.X, Tolerance);
        Assert.Equal(106.49929969569432, hit.CastPosition.Y, Tolerance);
        Assert.Equal(0.7612438726991966, hit.InterceptionTime, Tolerance);
    }
}
