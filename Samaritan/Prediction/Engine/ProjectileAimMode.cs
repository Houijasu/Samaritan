namespace Samaritan.Prediction.Engine;

/// <summary>
/// How projectile skillshots (linear, arc) are aimed against moving targets.
/// Placed effects (circular, rectangle, vector) always center on the target.
/// </summary>
public enum ProjectileAimMode
{
    /// <summary>
    /// The default: first contact at the EXACT (minimal) interception time, with
    /// the contact point swung toward the rear rim of the hitbox. Contact time is
    /// flat around its minimum over ray angles, so the rear swing - powered by
    /// the launch cushion (network compensation budgeted by the prediction but
    /// not waited for by the actual cast) - costs zero time. Falls back to the
    /// rear-edge tangency graze when the construction degenerates.
    /// </summary>
    RearGraze,

    /// <summary>
    /// The most tangent hit possible at any geometry: searches the missile ray
    /// angle directly for the ray whose closest approach to the target center is
    /// R·(1 - epsilon) in the actual-cast frame (raw delay - the prediction
    /// budgets network compensation that the real cast does not wait for), so
    /// the simulated "HIT by x" margin is minimal while still connecting. The
    /// contact lands on the rear side of the hitbox. Falls back to
    /// <see cref="RearGraze"/> when no in-range tangent ray exists. For live use
    /// with a server that matches the compensated timing, pass netComp = 0 to
    /// the solver instead.
    /// </summary>
    NearestRear,

    /// <summary>
    /// Combines <see cref="RearGraze"/> with <see cref="NearestRear"/>: among all
    /// rays whose first contact lands on the rear half of the hitbox, picks the
    /// one with the earliest first contact - the smallest interception time that
    /// still hits from behind - and casts at the contact point itself (closest
    /// cast position to the target). Falls back to <see cref="RearGraze"/> when
    /// no such ray exists.
    /// </summary>
    Optimal
}
