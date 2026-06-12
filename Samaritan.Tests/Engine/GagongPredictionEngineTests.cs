namespace Samaritan.Tests.Engine;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Collision;
using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Tests for the Gagong engine - a faithful port of a community Lua prediction
/// routine (per-segment quadratic, delay handled by path advancing, and a
/// width-exploiting bisection that pulls the hit earlier along the path).
/// Assertions are deliberately behavioral, not exact-value: the port preserves
/// the original's heuristics.
/// </summary>
public class GagongPredictionEngineTests
{
    private const double ProjectileSpeed = 1300;
    private const double Delay = 0.25;
    private const double EffectiveRadius = 85; // width/2 + hitbox = 20 + 65
    private const double HitboxRadius = 65;

    private static readonly Point2D Caster = new(0, 0);

    private static Skillshot.Linear NidaleeQ => new(
        Delay: (float)Delay, Speed: (float)ProjectileSpeed, Width: 40, Range: 1500);

    [Fact]
    public void PredictFromState_WalkingPerpendicular_HitsInSimulation()
    {
        var targetStart = new Point2D(600, -200);
        var velocity = new Vector2D(0, 350);
        var target = new MovementState.Walking(targetStart, velocity, null);

        var result = new GagongPredictionEngine().PredictFromState(
            NidaleeQ, Caster, target, HitboxRadius);
        var hit = Assert.IsType<PredictionResult.Hit>(result);

        var margin = EffectiveRadius - SimulatedMinGap(hit.CastPosition, targetStart, velocity);
        Assert.InRange(margin, 0.05, EffectiveRadius);
        Assert.True(hit.InterceptionTime > Delay, "Interception must happen after the cast delay");
    }

    [Fact]
    public void PredictFromState_ZigzagPath_FindsInterceptionOnPath()
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

        var result = new GagongPredictionEngine().PredictFromState(
            NidaleeQ, Caster, pathing, HitboxRadius);

        Assert.IsType<PredictionResult.Hit>(result);
    }

    [Fact]
    public void PredictFromState_StationaryTarget_ReturnsHit()
    {
        var target = new MovementState.Idle(new Point2D(800, 0));

        var result = new GagongPredictionEngine().PredictFromState(
            NidaleeQ, Caster, target, HitboxRadius);
        var hit = Assert.IsType<PredictionResult.Hit>(result);

        // Near-edge flight: delay + netComp + (distance - reach) / speed
        var expectedTime = Delay + 1.0 / 60 + (800 - EffectiveRadius) / ProjectileSpeed;
        Assert.Equal(expectedTime, hit.InterceptionTime, precision: 2);
    }

    [Fact]
    public void PredictFromState_TargetFarBeyondRange_ReturnsOutOfRange()
    {
        var target = new MovementState.Walking(new Point2D(1900, 0), new Vector2D(350, 0), null);

        var result = new GagongPredictionEngine().PredictFromState(
            NidaleeQ, Caster, target, HitboxRadius);

        Assert.IsType<PredictionResult.OutOfRange>(result);
    }

    /// <summary>
    /// Continuous minimum gap between missile front and target center in the
    /// simulation's frame (raw-delay launch).
    /// </summary>
    private static double SimulatedMinGap(
        Point2D castPosition, Point2D targetStart, Vector2D velocity)
    {
        var ray = (castPosition - Caster).Normalize();
        var origin = new Point2D(0, 0);
        var minGap = double.MaxValue;

        var previousOffset = (Caster - (targetStart + velocity.ScaleBy(Delay))).ToPoint2D();
        for (var t = Delay + 0.001; t <= Delay + 2.5; t += 0.001)
        {
            var front = Caster + ray.ScaleBy(ProjectileSpeed * (t - Delay));
            var center = targetStart + velocity.ScaleBy(t);
            var offset = (front - center).ToPoint2D();

            minGap = Math.Min(
                minGap,
                LinearCollisionDetector.PointToSegmentDistance(origin, previousOffset, offset));
            previousOffset = offset;
        }

        return minGap;
    }
}
