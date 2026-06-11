namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Interface for skillshot-specific collision detection.
/// </summary>
public interface ICollisionDetector
{
    /// <summary>
    /// Determines if a skillshot is hitting a target at the specified moment.
    /// </summary>
    /// <param name="skillshot">The skillshot to check.</param>
    /// <param name="origin">Skillshot origin (caster position).</param>
    /// <param name="aimPosition">Position the skillshot is aimed at. Defines both the
    /// direction and, for placed effects (circular, rectangle, vector), the landing point.</param>
    /// <param name="targetPosition">Target's position at <paramref name="timeElapsed"/>.</param>
    /// <param name="targetHitboxRadius">Target's hitbox radius.</param>
    /// <param name="timeElapsed">Time since skillshot was cast.</param>
    /// <returns>True if the skillshot hits the target.</returns>
    bool WillHit(
        Skillshot skillshot,
        Point2D origin,
        Point2D aimPosition,
        Point2D targetPosition,
        double targetHitboxRadius,
        double timeElapsed);
}
