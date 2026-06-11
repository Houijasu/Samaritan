namespace Samaritan.Tests.Engine;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Configuration;
using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Regression tests for the moving-target interception solver.
/// The original trailing-edge correction had a pole (division blow-up) at
/// ordinary approach angles, producing infeasible hits on one side of the
/// pole and spurious Unreachable results on the other.
/// </summary>
public class MovingTargetInterceptionTests
{
    // Infinite tick rate + zero ping makes NetworkCompensationDelay zero,
    // so effective delay equals the skillshot delay and the math is assertable.
    private static PredictionConfig ZeroLatencyConfig => new() { ServerTickRateHz = double.PositiveInfinity };

    /// <summary>
    /// Pole setup: delay 0.25s, target speed 380 => delay*speed == effectiveRadius (95),
    /// so hitboxRatio = 0.5 and the old correction divisor crossed zero at cos = 0.75.
    /// A 2000 u/s projectile vs a 380 u/s walker at 500 units is hittable at every angle.
    /// </summary>
    [Theory]
    [InlineData(0.70)]
    [InlineData(0.74)]
    [InlineData(0.7499)]
    [InlineData(0.75)]
    [InlineData(0.7501)]
    [InlineData(0.76)]
    [InlineData(0.80)]
    public void PredictFromState_ApproachAngleSweepAcrossFormerPole_AlwaysHits(double cosTheta)
    {
        var engine = new PredictionEngine(ZeroLatencyConfig, enableCaching: false);
        var skillshot = new Skillshot.Linear(Delay: 0.25f, Speed: 2000, Width: 60, Range: 2000);
        var caster = new Point2D(0, 0);

        var sinTheta = Math.Sqrt(1 - cosTheta * cosTheta);
        var velocity = new Vector2D(380 * cosTheta, 380 * sinTheta);
        var target = new MovementState.Walking(new Point2D(500, 0), velocity, null);

        var result = engine.PredictFromState(skillshot, caster, target, hitboxRadius: 65);

        Assert.IsType<PredictionResult.Hit>(result);
    }

    /// <summary>
    /// Projectile skillshots aim BEHIND the target: the cast position trails the
    /// predicted center along the movement direction by roughly the effective
    /// radius, so the missile grazes the rear edge of the hitbox as the target
    /// passes over the impact point (dodge-resistant aim - a target that stops
    /// still gets hit center-mass).
    /// </summary>
    [Fact]
    public void PredictFromState_LinearVsMovingTarget_AimsBehindTargetAlongMovementPath()
    {
        const double EffectiveRadius = 95; // width/2 + hitbox = 30 + 65

        var engine = new PredictionEngine(ZeroLatencyConfig, enableCaching: false);
        var skillshot = new Skillshot.Linear(Delay: 0.25f, Speed: 2000, Width: 60, Range: 2000);
        var caster = new Point2D(0, 0);

        // Target moving straight up (+Y); the aim point must trail it straight down
        var target = new MovementState.Walking(new Point2D(500, 0), new Vector2D(0, 300), null);

        var result = engine.PredictFromState(skillshot, caster, target, hitboxRadius: 65);

        var hit = Assert.IsType<PredictionResult.Hit>(result);

        var trailingOffset = hit.PredictedPosition - hit.CastPosition;
        Assert.Equal(0, trailingOffset.X, precision: 0); // purely along the path
        Assert.InRange(trailingOffset.Y, EffectiveRadius * 0.4, EffectiveRadius * 1.6);
    }

    /// <summary>
    /// The reported interception time must be the FIRST moment the missile front
    /// touches the hitbox, and the contact must land behind the target's center.
    /// This is the crossing geometry (far target cutting toward the line of fire)
    /// where a plain trailing-point solve clips the caster-side flank well before
    /// the reported time.
    /// </summary>
    [Fact]
    public void PredictFromState_CrossingTarget_ReportedTimeIsFirstContactBehindCenter()
    {
        const double ProjectileSpeed = 1200;
        const double Delay = 0.25;
        const double EffectiveRadius = 100; // 70/2 + 65

        var engine = new PredictionEngine(ZeroLatencyConfig, enableCaching: false);
        var skillshot = new Skillshot.Linear(Delay: (float)Delay, Speed: (float)ProjectileSpeed, Width: 70, Range: 1175);
        var caster = new Point2D(0, 0);

        // Far target crossing toward the caster's line of fire
        var targetStart = new Point2D(1000, 0);
        var velocity = new Vector2D(-120, 330);
        var target = new MovementState.Walking(targetStart, velocity, null);

        var result = engine.PredictFromState(skillshot, caster, target, hitboxRadius: 65);
        var hit = Assert.IsType<PredictionResult.Hit>(result);

        // Scan for the true first moment the missile front touches the hitbox
        var rayDirection = (hit.CastPosition - caster).Normalize();
        double? firstContact = null;
        for (var t = Delay; t <= hit.InterceptionTime + 0.5; t += 0.001)
        {
            var front = caster + rayDirection.ScaleBy(ProjectileSpeed * (t - Delay));
            var center = targetStart + velocity.ScaleBy(t);
            if (front.DistanceTo(center) <= EffectiveRadius)
            {
                firstContact = t;
                break;
            }
        }

        Assert.NotNull(firstContact);
        Assert.True(
            Math.Abs(firstContact.Value - hit.InterceptionTime) < 0.01,
            $"True first contact at {firstContact:F3}s but reported {hit.InterceptionTime:F3}s - missile clips the flank early");

        // The contact point must be behind the center relative to its movement
        var frontAtContact = caster + rayDirection.ScaleBy(ProjectileSpeed * (hit.InterceptionTime - Delay));
        var centerAtContact = targetStart + velocity.ScaleBy(hit.InterceptionTime);
        Assert.True(
            (frontAtContact - centerAtContact).DotProduct(velocity) < 0,
            "Contact must land behind the target's center, not on its leading side");
    }

    /// <summary>
    /// Every Hit must be physically consistent: at the reported interception time
    /// the missile front (launched after the delay, flying toward the cast
    /// position) is touching the hitbox - within the effective radius of the
    /// predicted target position, and never deeper than the graze margin allows.
    /// </summary>
    [Fact]
    public void PredictFromState_MovingTargetsAtAllAngles_HitsSatisfyTouchInvariant()
    {
        const double ProjectileSpeed = 2000;
        const double Delay = 0.25;
        const double HitboxRadius = 65;
        const double EffectiveRadius = 95; // width/2 + hitbox = 30 + 65

        var skillshot = new Skillshot.Linear(Delay: (float)Delay, Speed: (float)ProjectileSpeed, Width: 60, Range: 2000);
        var caster = new Point2D(0, 0);

        for (var angleDegrees = 0; angleDegrees < 360; angleDegrees += 15)
        {
            foreach (var targetSpeed in new[] { 200.0, 400.0 })
            {
                var angle = angleDegrees * Math.PI / 180.0;
                var velocity = new Vector2D(targetSpeed * Math.Cos(angle), targetSpeed * Math.Sin(angle));
                var target = new MovementState.Walking(new Point2D(500, 0), velocity, null);

                var engine = new PredictionEngine(ZeroLatencyConfig, enableCaching: false);
                var result = engine.PredictFromState(skillshot, caster, target, HitboxRadius);

                var hit = Assert.IsType<PredictionResult.Hit>(result);

                var rayDirection = (hit.CastPosition - caster).Normalize();
                var front = caster + rayDirection.ScaleBy(ProjectileSpeed * (hit.InterceptionTime - Delay));
                var gap = front.DistanceTo(hit.PredictedPosition);

                Assert.True(
                    gap <= EffectiveRadius + 3.0,
                    $"No contact at angle {angleDegrees}, speed {targetSpeed}: " +
                    $"missile front is {gap:F1} from the target center (R={EffectiveRadius}, T={hit.InterceptionTime:F4})");
            }
        }
    }

    /// <summary>
    /// When an interception solution exists but its aim point lies beyond the
    /// skillshot's range, the result is OutOfRange - not Unreachable.
    /// </summary>
    [Fact]
    public void PredictFromState_InterceptionBeyondRange_ReturnsOutOfRange()
    {
        var engine = new PredictionEngine(ZeroLatencyConfig, enableCaching: false);
        var skillshot = new Skillshot.Linear(Delay: 0.25f, Speed: 1200, Width: 60, Range: 750);
        var caster = new Point2D(0, 0);

        // Target at 700 units running directly away; interception happens near 800 units
        var target = new MovementState.Walking(new Point2D(700, 0), new Vector2D(300, 0), null);

        var result = engine.PredictFromState(skillshot, caster, target, hitboxRadius: 65);

        Assert.IsType<PredictionResult.OutOfRange>(result);
    }

    /// <summary>
    /// A target already inside the effective radius at launch is hit the moment
    /// the projectile fires.
    /// </summary>
    [Fact]
    public void PredictFromState_TargetInsideEffectiveRadiusAtLaunch_HitsAtDelay()
    {
        var engine = new PredictionEngine(ZeroLatencyConfig, enableCaching: false);
        var skillshot = new Skillshot.Linear(Delay: 0.25f, Speed: 2000, Width: 200, Range: 2000);
        var caster = new Point2D(0, 0);

        // Target 50 units away moving slowly; effective radius is 100 + 65 = 165
        var target = new MovementState.Walking(new Point2D(50, 0), new Vector2D(0, 100), null);

        var result = engine.PredictFromState(skillshot, caster, target, hitboxRadius: 65);

        var hit = Assert.IsType<PredictionResult.Hit>(result);
        Assert.Equal(0.25, hit.InterceptionTime, precision: 3);
    }
}
