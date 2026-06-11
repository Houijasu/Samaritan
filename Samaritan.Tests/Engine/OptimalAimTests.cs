namespace Samaritan.Tests.Engine;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Collision;
using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Optimal aim mode: combines AFTER (rear-side contact) with NEAREST (cast close
/// to the target). Among all rays whose first contact lands on the rear half of
/// the hitbox, it picks the earliest first contact - the smallest interception
/// time that still hits from behind - and casts at the contact point itself.
/// </summary>
public class OptimalAimTests
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
    public void PredictFromState_OptimalMode_ContactEarlierThanAfterAndNearest()
    {
        var targetStart = new Point2D(1000, 0);
        var velocity = new Vector2D(0, 350);
        var target = new MovementState.Walking(targetStart, velocity, null);
        var engine = NewEngine();

        var optimalHit = Assert.IsType<PredictionResult.Hit>(engine.PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.Optimal));
        var rearGrazeHit = Assert.IsType<PredictionResult.Hit>(engine.PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.RearGraze));
        var nearestHit = Assert.IsType<PredictionResult.Hit>(engine.PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.NearestRear));

        var optimalContact = SimulatedFirstContact(optimalHit.CastPosition, targetStart, velocity);
        var rearGrazeContact = SimulatedFirstContact(rearGrazeHit.CastPosition, targetStart, velocity);
        var nearestContact = SimulatedFirstContact(nearestHit.CastPosition, targetStart, velocity);

        Assert.NotNull(optimalContact);
        Assert.NotNull(rearGrazeContact);
        Assert.NotNull(nearestContact);

        Assert.True(
            optimalContact.Value < rearGrazeContact.Value - 0.003,
            $"Optimal contact {optimalContact:F3}s must beat rear graze {rearGrazeContact:F3}s");
        Assert.True(
            optimalContact.Value < nearestContact.Value - 0.003,
            $"Optimal contact {optimalContact:F3}s must beat nearest {nearestContact:F3}s");
    }

    [Fact]
    public void PredictFromState_OptimalMode_ContactIsOnRearHalf()
    {
        var targetStart = new Point2D(1000, 0);
        var velocity = new Vector2D(0, 350);
        var target = new MovementState.Walking(targetStart, velocity, null);

        var hit = Assert.IsType<PredictionResult.Hit>(NewEngine().PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.Optimal));

        var contactTime = SimulatedFirstContact(hit.CastPosition, targetStart, velocity);
        Assert.NotNull(contactTime);

        var ray = (hit.CastPosition - Caster).Normalize();
        var front = Caster + ray.ScaleBy(ProjectileSpeed * (contactTime.Value - Delay));
        var center = targetStart + velocity.ScaleBy(contactTime.Value);

        // Rear half (broadside tolerated): the contact must not be on the leading side
        var rearDot = (front - center).DotProduct(velocity);
        Assert.True(
            rearDot <= 0.05 * EffectiveRadius * velocity.Length,
            $"Contact must be on the rear half (rearDot = {rearDot:F0})");
    }

    [Fact]
    public void PredictFromState_OptimalMode_CastsCloseToTarget()
    {
        var targetStart = new Point2D(1000, 0);
        var velocity = new Vector2D(0, 350);
        var target = new MovementState.Walking(targetStart, velocity, null);

        var hit = Assert.IsType<PredictionResult.Hit>(NewEngine().PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.Optimal));

        // The cast position is the contact point - on or near the hitbox rim
        Assert.InRange(hit.CastPosition.DistanceTo(hit.PredictedPosition), 70, 130);
    }

    [Fact]
    public void PredictFromState_OptimalMode_HeadOn_HitsEarliest()
    {
        // Head-on, a rear first-contact is physically impossible (the target runs
        // into the missile face-first); Optimal must take the fastest hit instead.
        var targetStart = new Point2D(1000, 0);
        var velocity = new Vector2D(-350, 0);
        var target = new MovementState.Walking(targetStart, velocity, null);
        var engine = NewEngine();

        var optimalHit = Assert.IsType<PredictionResult.Hit>(engine.PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.Optimal));
        var nearestHit = Assert.IsType<PredictionResult.Hit>(engine.PredictFromState(
            MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.NearestRear));

        var optimalContact = SimulatedFirstContact(optimalHit.CastPosition, targetStart, velocity);
        var nearestContact = SimulatedFirstContact(nearestHit.CastPosition, targetStart, velocity);

        Assert.NotNull(optimalContact);
        Assert.NotNull(nearestContact);
        Assert.True(
            optimalContact.Value <= nearestContact.Value + 0.001,
            $"Optimal head-on contact {optimalContact:F3}s must not be later than nearest {nearestContact:F3}s");
    }

    [Fact]
    public void PredictFromState_OptimalMode_PenetrationStaysWithinHitboxRadius()
    {
        // The "HIT by" depth (R - minGap) must stay below the target's bounding
        // radius at every approach angle - fast, but never a through-center pass
        var targetStart = new Point2D(700, 0);

        for (var angleDegrees = 0; angleDegrees < 360; angleDegrees += 30)
        {
            var angle = angleDegrees * Math.PI / 180.0;
            var velocity = new Vector2D(350 * Math.Cos(angle), 350 * Math.Sin(angle));
            var target = new MovementState.Walking(targetStart, velocity, null);

            var result = NewEngine().PredictFromState(
                MorganaQ, Caster, target, HitboxRadius, ProjectileAimMode.Optimal);
            var hit = Assert.IsType<PredictionResult.Hit>(result);

            var minGap = SimulatedMinGap(hit.CastPosition, targetStart, velocity);
            var margin = EffectiveRadius - minGap;

            Assert.True(
                margin >= 0.05 && margin <= HitboxRadius,
                $"Angle {angleDegrees}: HIT-by {margin:F1} must stay within (0, {HitboxRadius}]");
        }
    }

    [Fact]
    public void PredictFromState_OptimalMode_FasterFleeingTarget_ReturnsUnreachable()
    {
        var slowSkillshot = new Skillshot.Linear(Delay: 0.25f, Speed: 300, Width: 70, Range: 1175);
        var target = new MovementState.Walking(new Point2D(400, 0), new Vector2D(350, 0), null);

        var result = NewEngine().PredictFromState(
            slowSkillshot, Caster, target, HitboxRadius, ProjectileAimMode.Optimal);

        Assert.IsType<PredictionResult.Unreachable>(result);
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

    /// <summary>
    /// First moment the missile front touches the hitbox in the simulation's
    /// frame (raw-delay launch), using the same segment-based relative-motion
    /// math as the simulator's swept collision check.
    /// </summary>
    private static double? SimulatedFirstContact(
        Point2D castPosition, Point2D targetStart, Vector2D velocity)
    {
        var ray = (castPosition - Caster).Normalize();
        var origin = new Point2D(0, 0);

        var previousOffset = (Caster - (targetStart + velocity.ScaleBy(Delay))).ToPoint2D();
        for (var t = Delay + 0.001; t <= Delay + 2.5; t += 0.001)
        {
            var front = Caster + ray.ScaleBy(ProjectileSpeed * (t - Delay));
            var center = targetStart + velocity.ScaleBy(t);
            var offset = (front - center).ToPoint2D();

            if (LinearCollisionDetector.PointToSegmentDistance(origin, previousOffset, offset) <= EffectiveRadius)
                return t;

            previousOffset = offset;
        }

        return null;
    }
}
