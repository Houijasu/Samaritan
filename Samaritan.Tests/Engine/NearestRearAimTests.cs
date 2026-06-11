namespace Samaritan.Tests.Engine;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Collision;
using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// NearestRear aim mode: the most tangent hit possible at ANY geometry.
/// The solver searches over the missile ray angle directly for the ray whose
/// closest approach to the target center equals R*(1-epsilon) in the simulation
/// frame (raw-delay launch) - the exact quantity the HUD "Graze" line measures -
/// so the simulated "HIT by x" margin is ~0.25-0.35 units wherever a tangent
/// graze is physically reachable, falling back to the rear-graze aim otherwise.
/// </summary>
public class NearestRearAimTests
{
    private const double ProjectileSpeed = 1200;
    private const double Delay = 0.25;
    private const double EffectiveRadius = 100; // width/2 + hitbox = 35 + 65
    private const double HitboxRadius = 65;

    private static readonly Point2D Caster = new(0, 0);

    private static Skillshot.Linear MorganaQ => new(
        Delay: (float)Delay, Speed: (float)ProjectileSpeed, Width: 70, Range: 1175);

    private static PredictionEngine NewEngine() => new(enableCaching: false);

    [Fact]
    public void PredictFromState_NearestRearMode_CastsOnRearRimOfHitbox()
    {
        var targetStart = new Point2D(1000, 0);
        var velocity = new Vector2D(0, 350);
        var target = new MovementState.Walking(targetStart, velocity, null);

        var result = NewEngine().PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.NearestRear);
        var hit = Assert.IsType<PredictionResult.Hit>(result);

        // The cast position is the first-contact point on the hitbox rim
        // (within frame-mixing slack of v*netComp plus the graze chord)
        Assert.InRange(hit.CastPosition.DistanceTo(hit.PredictedPosition), 85, 115);

        // ...on the rear side relative to the movement direction
        Assert.True(
            (hit.CastPosition - hit.PredictedPosition).DotProduct(velocity) < 0,
            "Cast position must be behind the predicted center");
    }

    [Fact]
    public void PredictFromState_NearestRearMode_HitsWithTinyMargin()
    {
        var targetStart = new Point2D(1000, 0);
        var velocity = new Vector2D(0, 350);
        var target = new MovementState.Walking(targetStart, velocity, null);

        var hit = Assert.IsType<PredictionResult.Hit>(NewEngine().PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.NearestRear));

        var margin = EffectiveRadius - SimulatedMinGap(hit.CastPosition, targetStart, velocity);

        Assert.InRange(margin, 0.05, 2.5);
    }

    [Fact]
    public void PredictFromState_NearestRearMode_IsMoreTangentThanRearGraze()
    {
        var targetStart = new Point2D(1000, 0);
        var velocity = new Vector2D(0, 350);
        var target = new MovementState.Walking(targetStart, velocity, null);
        var engine = NewEngine();

        var rearGrazeHit = Assert.IsType<PredictionResult.Hit>(engine.PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.RearGraze));
        var nearestHit = Assert.IsType<PredictionResult.Hit>(engine.PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.NearestRear));

        var rearGrazeMargin = EffectiveRadius - SimulatedMinGap(rearGrazeHit.CastPosition, targetStart, velocity);
        var nearestMargin = EffectiveRadius - SimulatedMinGap(nearestHit.CastPosition, targetStart, velocity);

        Assert.True(rearGrazeMargin > 0, "Rear graze must hit");
        Assert.True(
            nearestMargin < rearGrazeMargin - 1.0,
            $"NearestRear margin {nearestMargin:F2} must be clearly smaller than rear-graze margin {rearGrazeMargin:F2}");
    }

    [Fact]
    public void PredictFromState_DefaultMode_IsUnchangedRearGraze()
    {
        var target = new MovementState.Walking(new Point2D(1000, 0), new Vector2D(0, 350), null);
        var engine = NewEngine();

        var defaultResult = engine.PredictFromState(MorganaQ, Caster, target, HitboxRadius);
        var rearGrazeResult = engine.PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.RearGraze);

        Assert.Equal(defaultResult, rearGrazeResult);
    }

    /// <summary>
    /// Regression for the shallow-crossing failure (user-measured "HIT by 53"):
    /// the old lead-based solver capped the tangency lead at 2R, forcing deep
    /// penetration when sin(phi) is small.
    /// </summary>
    [Fact]
    public void PredictFromState_NearestRearMode_ShallowCrossing_StaysTangent()
    {
        // Target offset matches the user's geometry; direction 12 degrees off
        // the caster ray => sin(phi) ~ 0.2 at the solution
        var targetStart = new Point2D(480, 456);
        var velocity = new Vector2D(198.2, 288.7); // 350 u/s, 12 deg off the ray

        var target = new MovementState.Walking(targetStart, velocity, null);
        var hit = Assert.IsType<PredictionResult.Hit>(NewEngine().PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.NearestRear));

        var margin = EffectiveRadius - SimulatedMinGap(hit.CastPosition, targetStart, velocity);

        Assert.InRange(margin, 0.05, 2.5);
    }

    [Fact]
    public void PredictFromState_NearestRearMode_AngleSweep_AlwaysTangentHit()
    {
        var targetStart = new Point2D(700, 0);

        for (var angleDegrees = 0; angleDegrees < 360; angleDegrees += 10)
        {
            var angle = angleDegrees * Math.PI / 180.0;
            var velocity = new Vector2D(350 * Math.Cos(angle), 350 * Math.Sin(angle));
            var target = new MovementState.Walking(targetStart, velocity, null);

            var result = NewEngine().PredictFromState(
                MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.NearestRear);
            var hit = Assert.IsType<PredictionResult.Hit>(result);

            var margin = EffectiveRadius - SimulatedMinGap(hit.CastPosition, targetStart, velocity);

            Assert.True(
                margin >= 0.05 && margin <= 2.5,
                $"Angle {angleDegrees}: margin {margin:F2} outside the tangent band");
        }
    }

    [Fact]
    public void PredictFromState_NearestRearMode_HeadOn_BroadsideGraze()
    {
        var targetStart = new Point2D(1000, 0);
        var velocity = new Vector2D(-350, 0);

        var target = new MovementState.Walking(targetStart, velocity, null);
        var hit = Assert.IsType<PredictionResult.Hit>(NewEngine().PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.NearestRear));

        var margin = EffectiveRadius - SimulatedMinGap(hit.CastPosition, targetStart, velocity);

        Assert.InRange(margin, 0.05, 2.5);
    }

    [Fact]
    public void PredictFromState_NearestRearMode_Chase_SideGraze()
    {
        var targetStart = new Point2D(400, 0);
        var velocity = new Vector2D(350, 0);

        var target = new MovementState.Walking(targetStart, velocity, null);
        var hit = Assert.IsType<PredictionResult.Hit>(NewEngine().PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.NearestRear));

        var margin = EffectiveRadius - SimulatedMinGap(hit.CastPosition, targetStart, velocity);

        Assert.InRange(margin, 0.05, 2.5);
    }

    [Fact]
    public void PredictFromState_NearestRearMode_InterceptBeyondRange_ReturnsOutOfRange()
    {
        // Fleeing target whose interception point lies past the skillshot range
        var target = new MovementState.Walking(new Point2D(1100, 0), new Vector2D(350, 0), null);

        var result = NewEngine().PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.NearestRear);

        Assert.IsType<PredictionResult.OutOfRange>(result);
    }

    [Fact]
    public void PredictFromState_NearestRearMode_FasterFleeingTarget_ReturnsUnreachable()
    {
        var slowSkillshot = new Skillshot.Linear(Delay: 0.25f, Speed: 300, Width: 70, Range: 1175);
        var target = new MovementState.Walking(new Point2D(400, 0), new Vector2D(350, 0), null);

        var result = NewEngine().PredictFromState(
            slowSkillshot, Caster, target, HitboxRadius, ProjectileAimMode.NearestRear);

        Assert.IsType<PredictionResult.Unreachable>(result);
    }

    [Fact]
    public void PredictFromState_NearestRearMode_TargetInsideRadiusAtLaunch_HitsAtDelay()
    {
        var wideSkillshot = new Skillshot.Linear(Delay: 0.25f, Speed: 1200, Width: 200, Range: 1175);
        var target = new MovementState.Walking(new Point2D(50, 0), new Vector2D(0, 100), null);

        var result = NewEngine().PredictFromState(
            wideSkillshot, Caster, target, HitboxRadius, ProjectileAimMode.NearestRear);
        var hit = Assert.IsType<PredictionResult.Hit>(result);

        var effectiveDelay = Delay + PredictionConfigNetComp();
        Assert.Equal(effectiveDelay, hit.InterceptionTime, precision: 3);
    }

    private static double PredictionConfigNetComp() =>
        Samaritan.Prediction.Configuration.PredictionConfig.Default.NetworkCompensationDelay;

    /// <summary>
    /// Continuous minimum gap between the missile front and the target center in
    /// the simulation's frame (raw-delay launch), using the same segment-based
    /// relative-motion math as the simulator's swept collision check.
    /// </summary>
    private static double SimulatedMinGap(
        Point2D castPosition,
        Point2D targetStart,
        Vector2D velocity,
        double delay = Delay,
        double speed = ProjectileSpeed,
        double horizon = 2.5)
    {
        var ray = (castPosition - Caster).Normalize();
        var origin = new Point2D(0, 0);
        var minGap = double.MaxValue;

        var previousOffset = (Caster - (targetStart + velocity.ScaleBy(delay))).ToPoint2D();
        for (var t = delay + 0.001; t <= delay + horizon; t += 0.001)
        {
            var front = Caster + ray.ScaleBy(speed * (t - delay));
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

internal static class VectorTestExtensions
{
    public static Point2D ToPoint2D(this Vector2D vector) => new(vector.X, vector.Y);
}
