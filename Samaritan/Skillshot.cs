namespace Samaritan;

using Dunet;

/// <summary>
/// Represents a skillshot type in League of Legends.
/// </summary>
[Union]
public partial record Skillshot
{
    /// <summary>
    /// A linear skillshot that travels in a straight line.
    /// Examples: Ezreal Q, Nidalee Q, Morgana Q
    /// </summary>
    /// <param name="Delay">The cast delay in seconds before the projectile fires (0 for instant).</param>
    /// <param name="Speed">The projectile speed (units per second).</param>
    /// <param name="Width">The width of the skillshot.</param>
    /// <param name="Range">The maximum travel distance.</param>
    public partial record Linear(float Delay, float Speed, float Width, float Range);

    /// <summary>
    /// A circular skillshot that affects an area.
    /// Examples: Lux E, Ziggs Q, Veigar W
    /// </summary>
    /// <param name="Delay">The delay in seconds before impact (0 for instant).</param>
    /// <param name="Speed">The projectile speed to target location (units per second).</param>
    /// <param name="Radius">The radius of the affected area.</param>
    /// <param name="Range">The maximum cast range from origin.</param>
    public partial record Circular(float Delay, float Speed, float Radius, float Range);

    /// <summary>
    /// A cone-shaped skillshot that spreads outward.
    /// Examples: Annie W, Cho'Gath W, Mordekaiser E
    /// </summary>
    /// <param name="Delay">The delay in seconds before activation (0 for instant).</param>
    /// <param name="Angle">The cone angle in degrees.</param>
    /// <param name="Range">The length of the cone.</param>
    public partial record Cone(float Delay, float Angle, float Range);

    /// <summary>
    /// An arc-shaped skillshot that curves in flight.
    /// Examples: Diana Q, Neeko Q, Seraphine R
    /// </summary>
    /// <param name="Delay">The delay in seconds before the arc begins (0 for instant).</param>
    /// <param name="Speed">The arc travel speed (units per second).</param>
    /// <param name="Width">The width of the arc.</param>
    /// <param name="OuterRadius">The outer radius of the arc curve.</param>
    /// <param name="Angle">The arc angle in degrees.</param>
    /// <param name="Clockwise">True if arc curves clockwise, false for counter-clockwise (default).</param>
    public partial record Arc(float Delay, float Speed, float Width, float OuterRadius, float Angle, bool Clockwise = false);

    /// <summary>
    /// A rectangular skillshot that covers a rectangular area, cast from caster position.
    /// Examples: Anivia W, Karthus W
    /// </summary>
    /// <param name="Delay">The delay in seconds before activation (0 for instant).</param>
    /// <param name="Speed">The projectile speed to target location (units per second).</param>
    /// <param name="Width">The width of the rectangle.</param>
    /// <param name="Length">The length of the rectangle.</param>
    /// <param name="Range">The maximum cast range from origin.</param>
    public partial record Rectangle(float Delay, float Speed, float Width, float Length, float Range);

    /// <summary>
    /// A vector-cast rectangular skillshot with separate start and end points.
    /// Examples: Viktor E, Rumble R, Taliyah R
    /// </summary>
    /// <param name="Delay">The delay in seconds before activation (0 for instant).</param>
    /// <param name="Speed">The projectile speed along the vector (units per second).</param>
    /// <param name="Width">The width of the rectangle.</param>
    /// <param name="MaxLength">The maximum length of the vector cast.</param>
    /// <param name="Range">The maximum cast range for the start point from caster.</param>
    public partial record VectorRectangle(float Delay, float Speed, float Width, float MaxLength, float Range);
}

/// <summary>
/// Common parameter accessors shared by the prediction engine, solvers, and simulation.
/// </summary>
public static class SkillshotExtensions
{
    /// <summary>
    /// Gets the cast delay in seconds.
    /// </summary>
    public static double GetDelay(this Skillshot skillshot)
    {
        return skillshot.Match(
            linear: l => (double)l.Delay,
            circular: c => (double)c.Delay,
            cone: c => (double)c.Delay,
            arc: a => (double)a.Delay,
            rectangle: r => (double)r.Delay,
            vectorRectangle: v => (double)v.Delay);
    }

    /// <summary>
    /// Gets the maximum distance at which the skillshot can connect.
    /// </summary>
    public static double GetMaxRange(this Skillshot skillshot)
    {
        return skillshot.Match(
            linear: l => (double)l.Range,
            circular: c => (double)c.Range,
            cone: c => (double)c.Range,
            arc: a => (double)a.OuterRadius,
            rectangle: r => (double)r.Range,
            vectorRectangle: v => (double)(v.Range + v.MaxLength));
    }

    /// <summary>
    /// Gets the projectile travel speed, or null when the skillshot applies
    /// instantly (no travel time).
    /// </summary>
    public static double? GetProjectileSpeed(this Skillshot skillshot)
    {
        var speed = skillshot.Match(
            linear: l => (double)l.Speed,
            circular: c => (double)c.Speed,
            cone: _ => 0.0, // Instant area-of-effect
            arc: a => (double)a.Speed,
            rectangle: r => (double)r.Speed,
            vectorRectangle: v => (double)v.Speed);

        return speed > 0 ? speed : null;
    }

    /// <summary>
    /// Gets the effective hit radius: the distance from the target center at
    /// which this skillshot connects, given the target's hitbox radius.
    /// </summary>
    public static double GetEffectiveRadius(this Skillshot skillshot, double hitboxRadius)
    {
        return skillshot.Match(
            linear: l => l.Width / 2.0 + hitboxRadius,
            circular: c => c.Radius + hitboxRadius,
            cone: _ => hitboxRadius, // Cone has no width (instant area-of-effect from point)
            arc: a => a.Width / 2.0 + hitboxRadius,
            rectangle: r => r.Width / 2.0 + hitboxRadius,
            vectorRectangle: v => v.Width / 2.0 + hitboxRadius);
    }
}
