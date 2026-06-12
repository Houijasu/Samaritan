namespace Samaritan.Tests.Engine;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Collision;
using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Defining spec of the default (AFTER) aim mode: the actual hit lands at the
/// EXACT method's minimal interception time, with the contact point swung
/// toward the rear rim using the launch cushion - rear-ness at zero time cost
/// (the flat region around the time minimum).
/// </summary>
public class ExactTimeRearTests
{
    private const double ProjectileSpeed = 1200;
    private const double Delay = 0.25;
    private const double EffectiveRadius = 100; // width/2 + hitbox = 35 + 65
    private const double HitboxRadius = 65;

    private static readonly Point2D Caster = new(0, 0);
    private static readonly Point2D TargetStart = new(1000, 0);
    private static readonly Vector2D Velocity = new(-120, 330);

    private static Skillshot.Linear MorganaQ => new(
        Delay: (float)Delay, Speed: (float)ProjectileSpeed, Width: 70, Range: 1175);

    [Fact]
    public void PredictFromState_DefaultMode_ActualHitMatchesExactTime()
    {
        var engine = new PredictionEngine(enableCaching: false);
        var target = new MovementState.Walking(TargetStart, Velocity, null);

        var exactHit = Assert.IsType<PredictionResult.Hit>(
            engine.PredictExact(MorganaQ, Caster, target, HitboxRadius));
        var hit = Assert.IsType<PredictionResult.Hit>(
            engine.PredictFromState(MorganaQ, Caster, target, HitboxRadius));

        var actual = SimulatedFirstContact(hit.CastPosition);
        Assert.NotNull(actual);
        Assert.True(
            Math.Abs(actual.Value - exactHit.InterceptionTime) <= 0.004,
            $"Actual hit {actual:F4}s must equal the exact time {exactHit.InterceptionTime:F4}s");
    }

    [Fact]
    public void PredictFromState_DefaultMode_CastsOnRearRim()
    {
        var engine = new PredictionEngine(enableCaching: false);
        var target = new MovementState.Walking(TargetStart, Velocity, null);

        var hit = Assert.IsType<PredictionResult.Hit>(
            engine.PredictFromState(MorganaQ, Caster, target, HitboxRadius));

        // Cast position sits exactly on the hitbox rim at the predicted moment...
        Assert.Equal(EffectiveRadius, hit.CastPosition.DistanceTo(hit.PredictedPosition), precision: 0);

        // ...on the rear side (broadside tolerated)
        var rearDot = (hit.CastPosition - hit.PredictedPosition).DotProduct(Velocity);
        Assert.True(
            rearDot <= 0.05 * EffectiveRadius * Velocity.Length,
            $"Contact must not sit ahead of the center (rearDot = {rearDot:F0})");
    }

    /// <summary>
    /// First contact in the simulation's frame (raw-delay launch, one-tick swept
    /// segments, contact time interpolated like the simulator does).
    /// </summary>
    private static double? SimulatedFirstContact(Point2D castPosition)
    {
        var ray = (castPosition - Caster).Normalize();
        var origin = new Point2D(0, 0);

        var previous = (Caster - (TargetStart + Velocity.ScaleBy(Delay))).ToPoint2D();
        for (var t = Delay + 0.001; t <= Delay + 2.5; t += 0.001)
        {
            var front = Caster + ray.ScaleBy(ProjectileSpeed * (t - Delay));
            var center = TargetStart + Velocity.ScaleBy(t);
            var offset = (front - center).ToPoint2D();

            if (LinearCollisionDetector.SweptContact(
                    new Vector2D(previous.X, previous.Y),
                    new Vector2D(offset.X, offset.Y),
                    EffectiveRadius,
                    out var fraction))
            {
                return t - 0.001 + fraction * 0.001;
            }

            previous = offset;
        }

        return null;
    }
}
