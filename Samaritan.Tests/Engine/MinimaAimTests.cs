namespace Samaritan.Tests.Engine;

using System.Text;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Collision;
using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Minima aim mode: beats the Gagong reference on ACTUAL HIT - the contact
/// lands earlier by a share of Gagong's own slack above the global floor
/// (capped at 2 ms), tying the floor where Gagong is optimal - and then takes
/// the shallowest pass that fits under that contact budget. The battery below
/// pins the contract against the Gagong port on identical states.
/// </summary>
public class MinimaAimTests
{
    private const double ProjectileSpeed = 1300;
    private const double Delay = 0.25;
    private const double EffectiveRadius = 85; // width/2 + hitbox = 20 + 65
    private const double HitboxRadius = 65;

    // Contact-time tolerance: sweep discretization plus bisection slack
    private const double ContactTolerance = 0.0015;
    // Display-tie window: within this, the two contacts read as equal (F3 HUD)
    private const double TieTolerance = 0.00025;
    // Penetration tolerances, in units: near-tie noise, and the small extra
    // depth the contact margin buys on the contact-vs-depth frontier
    private const double PenetrationTolerance = 2.0;
    private const double PenetrationSlack = 4.0;

    private static readonly Point2D Caster = new(0, 0);

    private static Skillshot.Linear NidaleeQ => new(
        Delay: (float)Delay, Speed: (float)ProjectileSpeed, Width: 40, Range: 1500);

    private static PredictionEngine NewEngine() => new(enableCaching: false);

    [Fact]
    public void PredictFromState_MinimaMode_BeatsGagongOnBothMetrics_AllAngles()
    {
        // Contract: (1) ACTUAL HIT is never later than Gagong's - the hard
        // priority; (2) HIT BY never exceeds Gagong's by more than the small
        // depth the contact margin buys on the frontier; (3) a display tie
        // never comes with a meaningfully deeper pass; (4) the contact win is
        // real where Gagong has slack to take it from.
        var report = new StringBuilder();
        var violations = new List<string>();
        var bestWin = double.MinValue;

        for (var angleDegrees = 0; angleDegrees < 360; angleDegrees += 15)
        {
            var angle = angleDegrees * Math.PI / 180.0;
            var velocity = new Vector2D(350 * Math.Cos(angle), 350 * Math.Sin(angle));
            var targetStart = new Point2D(700, 0);
            var target = new MovementState.Walking(targetStart, velocity, null);

            var minimaResult = NewEngine().PredictFromState(
                NidaleeQ, Caster, target, HitboxRadius, ProjectileAimMode.Minima);
            Assert.True(minimaResult is PredictionResult.Hit, $"Angle {angleDegrees}: Minima must hit");
            var minimaHit = (PredictionResult.Hit)minimaResult;

            var gagongResult = new GagongPredictionEngine().PredictFromState(
                NidaleeQ, Caster, target, HitboxRadius);
            Assert.True(gagongResult is PredictionResult.Hit,
                $"Angle {angleDegrees}: Gagong must hit for the comparison to be meaningful");
            var gagongHit = (PredictionResult.Hit)gagongResult;

            var minimaContact = ExactFirstContact(minimaHit.CastPosition, targetStart, velocity);
            var gagongContact = ExactFirstContact(gagongHit.CastPosition, targetStart, velocity);
            Assert.NotNull(minimaContact);
            Assert.NotNull(gagongContact);

            var minimaPenetration = EffectiveRadius - SimulatedMinGap(minimaHit.CastPosition, targetStart, velocity);
            var gagongPenetration = EffectiveRadius - SimulatedMinGap(gagongHit.CastPosition, targetStart, velocity);

            report.AppendLine(
                $"Angle {angleDegrees,3}: contact minima {minimaContact:F4}s vs gagong {gagongContact:F4}s "
                + $"(diff {minimaContact.Value - gagongContact.Value:+0.0000;-0.0000}s); "
                + $"HIT-by minima {minimaPenetration:F1} vs gagong {gagongPenetration:F1}");

            // (1) ACTUAL HIT: never later than Gagong's contact
            if (minimaContact.Value > gagongContact.Value + ContactTolerance)
                violations.Add($"Angle {angleDegrees}: Minima contact {minimaContact:F4}s is later than Gagong {gagongContact:F4}s");

            // (2) HIT BY: at most a few units deeper than Gagong's
            if (minimaPenetration > gagongPenetration + PenetrationSlack)
                violations.Add($"Angle {angleDegrees}: Minima HIT-by {minimaPenetration:F1} exceeds Gagong {gagongPenetration:F1} by more than {PenetrationSlack}");

            // (3) A display tie must not come with a meaningfully deeper pass
            if (Math.Abs(minimaContact.Value - gagongContact.Value) <= TieTolerance
                && minimaPenetration > gagongPenetration + PenetrationTolerance)
            {
                violations.Add(
                    $"Angle {angleDegrees}: at tied contact ({minimaContact:F4}s vs {gagongContact:F4}s), "
                    + $"Minima HIT-by {minimaPenetration:F1} is deeper than Gagong {gagongPenetration:F1}");
            }

            bestWin = Math.Max(bestWin, gagongContact.Value - minimaContact.Value);
        }

        // (4) Where Gagong has slack above the floor, the margin must fire
        Assert.True(
            bestWin >= 0.001,
            $"Minima must beat Gagong's contact by at least 1 ms at some geometry (best win: {bestWin * 1000:F2} ms)");

        Assert.True(
            violations.Count == 0,
            $"Minima contract violated (ACTUAL HIT <= Gagong, HIT BY within a few units).{Environment.NewLine}"
            + string.Join(Environment.NewLine, violations) + Environment.NewLine + report);
    }

    [Fact]
    public void PredictFromState_MinimaMode_HeadOn_TiesGagongFloorWithoutDepthLoss()
    {
        // Head-on, Gagong's centered interception sits on the global contact
        // floor; Minima ties it and must not pass deeper than Gagong's
        // face-first pass
        var targetStart = new Point2D(1000, 0);
        var velocity = new Vector2D(-350, 0);
        var target = new MovementState.Walking(targetStart, velocity, null);

        var minimaHit = Assert.IsType<PredictionResult.Hit>(NewEngine().PredictFromState(
            NidaleeQ, Caster, target, HitboxRadius, ProjectileAimMode.Minima));
        var gagongHit = Assert.IsType<PredictionResult.Hit>(new GagongPredictionEngine().PredictFromState(
            NidaleeQ, Caster, target, HitboxRadius));

        var minimaContact = ExactFirstContact(minimaHit.CastPosition, targetStart, velocity);
        var gagongContact = ExactFirstContact(gagongHit.CastPosition, targetStart, velocity);
        Assert.NotNull(minimaContact);
        Assert.NotNull(gagongContact);

        var minimaPenetration = EffectiveRadius - SimulatedMinGap(minimaHit.CastPosition, targetStart, velocity);
        var gagongPenetration = EffectiveRadius - SimulatedMinGap(gagongHit.CastPosition, targetStart, velocity);

        Assert.True(
            minimaContact.Value <= gagongContact.Value + ContactTolerance,
            $"Head-on contact {minimaContact:F4}s must not be later than Gagong's floor {gagongContact:F4}s");
        Assert.True(
            minimaPenetration <= gagongPenetration + PenetrationTolerance,
            $"Head-on HIT-by {minimaPenetration:F1} must not be deeper than Gagong's {gagongPenetration:F1}");
    }

    [Fact]
    public void PredictFromState_MinimaMode_FasterFleeingTarget_ReturnsUnreachable()
    {
        var slowSkillshot = new Skillshot.Linear(Delay: 0.25f, Speed: 300, Width: 40, Range: 1500);
        var target = new MovementState.Walking(new Point2D(400, 0), new Vector2D(350, 0), null);

        var result = NewEngine().PredictFromState(
            slowSkillshot, Caster, target, HitboxRadius, ProjectileAimMode.Minima);

        Assert.IsType<PredictionResult.Unreachable>(result);
    }

    [Fact]
    public void PredictFromState_MinimaMode_StationaryTarget_ReturnsHit()
    {
        // Stationary targets take the standard solver branch, independent of the
        // aim mode - the mode only governs moving projectile skillshots
        var target = new MovementState.Idle(new Point2D(800, 0));

        var result = NewEngine().PredictFromState(
            NidaleeQ, Caster, target, HitboxRadius, ProjectileAimMode.Minima);

        var hit = Assert.IsType<PredictionResult.Hit>(result);
        Assert.True(hit.InterceptionTime > Delay, "Interception must happen after the cast delay");
    }

    /// <summary>
    /// Exact first-contact time in the simulation's frame (raw-delay launch):
    /// the smallest s >= 0 with |G0 + W*s| = R, where G0 is the center-minus-
    /// caster offset at launch and W the relative velocity along the cast ray.
    /// </summary>
    private static double? ExactFirstContact(
        Point2D castPosition, Point2D targetStart, Vector2D velocity)
    {
        var ray = (castPosition - Caster).Normalize();
        var launchOffset = (targetStart - Caster) + velocity.ScaleBy(Delay);
        var relativeVelocity = velocity - ray.ScaleBy(ProjectileSpeed);

        var a = relativeVelocity.DotProduct(relativeVelocity);
        var b = 2.0 * launchOffset.DotProduct(relativeVelocity);
        var c = launchOffset.DotProduct(launchOffset) - EffectiveRadius * EffectiveRadius;

        if (c <= 0) // already touching at launch
            return Delay;

        if (a < 1e-12)
            return Math.Abs(b) > 1e-12 && -c / b >= 0 ? Delay + (-c / b) : null;

        var discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0)
            return null;

        var sqrt = Math.Sqrt(discriminant);
        var root1 = (-b - sqrt) / (2.0 * a);
        var root2 = (-b + sqrt) / (2.0 * a);

        var flight = root1 >= 0 ? root1 : root2;
        return flight >= 0 ? Delay + flight : null;
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
