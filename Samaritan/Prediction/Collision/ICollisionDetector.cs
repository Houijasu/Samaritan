namespace Samaritan.Prediction.Collision;

using MathNet.Spatial.Euclidean;

/// <summary>
/// Interface for skillshot-specific collision detection.
/// </summary>
public interface ICollisionDetector
{
    /// <summary>
    /// Determines if a skillshot will hit a target at the specified position.
    /// </summary>
    /// <param name="skillshot">The skillshot to check.</param>
    /// <param name="origin">Skillshot origin (caster position).</param>
    /// <param name="aimDirection">Normalized direction the skillshot is aimed.</param>
    /// <param name="targetPosition">Target's position.</param>
    /// <param name="targetHitboxRadius">Target's hitbox radius.</param>
    /// <param name="timeElapsed">Time since skillshot was cast.</param>
    /// <returns>True if the skillshot hits the target.</returns>
    bool WillHit(
        Skillshot skillshot,
        Point2D origin,
        Vector2D aimDirection,
        Point2D targetPosition,
        double targetHitboxRadius,
        double timeElapsed);
}
