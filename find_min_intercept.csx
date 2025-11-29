#!/usr/bin/env dotnet-script
#r "nuget: MathNet.Spatial, 0.6.0"

using System;
using MathNet.Spatial.Euclidean;

// Scenario #9: LinearVsMaxRange
// Nidalee Q parameters
const double Delay = 0.25;
const double Speed = 1300.0;
const double Width = 40.0;
const double Range = 1500.0;

// Scenario setup
var casterPosition = new Point2D(0, 0);
var targetStart = new Point2D(1400, -200);
var targetVelocity = new Vector2D(50, 250);
const double HitboxRadius = 65.0;
const double CollisionThreshold = HitboxRadius + Width / 2.0; // 85

Point2D GetTargetPosition(double t) =>
    new(targetStart.X + targetVelocity.X * t, targetStart.Y + targetVelocity.Y * t);

// Analytical calculation of hit time given aim time T
(double minDist, double hitTime, double flightTime)? AnalyzeAimTime(double T)
{
    // Aim point A = T₀ + V·T
    var A = new Vector2D(targetStart.X + targetVelocity.X * T, targetStart.Y + targetVelocity.Y * T);
    var aimDist = A.Length;

    if (aimDist > Range) return null;

    // Unit direction û = A/|A|
    var u = A.Normalize();

    // W = s·û - V (relative velocity of projectile w.r.t. target)
    var W = new Vector2D(Speed * u.X - targetVelocity.X, Speed * u.Y - targetVelocity.Y);

    // B = T₀ + V·δ (target position when projectile launches)
    var B = new Vector2D(targetStart.X + targetVelocity.X * Delay, targetStart.Y + targetVelocity.Y * Delay);

    var W_dot_B = W.DotProduct(B);
    var W_sq = W.DotProduct(W);
    var B_sq = B.DotProduct(B);

    // Minimum distance squared: d²_min = |B|² - (W·B)²/|W|²
    var D_min_sq = B_sq - (W_dot_B * W_dot_B) / W_sq;
    var D_min = Math.Sqrt(Math.Max(0, D_min_sq));

    if (D_min > CollisionThreshold) return null; // No hit possible

    // Discriminant for hit time calculation
    var discriminant = W_dot_B * W_dot_B - W_sq * (B_sq - CollisionThreshold * CollisionThreshold);

    if (discriminant < 0) return null;

    // First hit time (smaller root = entry into hitbox)
    // τ_hit = [(W·B) - √discriminant] / |W|²
    var tau_hit = (W_dot_B - Math.Sqrt(discriminant)) / W_sq;

    if (tau_hit < 0) return null; // Hit would be before launch

    var t_hit = tau_hit + Delay;

    return (D_min, t_hit, tau_hit);
}

// Find T_center: standard interception (aim at center)
double FindTCenter()
{
    // Solve: s·(T - δ) = |T₀ + V·T|
    // Squaring: s²(T-δ)² = |T₀|² + 2(T₀·V)T + |V|²T²
    // (s² - |V|²)T² - (2s²δ + 2(T₀·V))T + (s²δ² - |T₀|²) = 0
    var s_sq = Speed * Speed;
    var v_sq = targetVelocity.DotProduct(targetVelocity);
    var T0_sq = targetStart.X * targetStart.X + targetStart.Y * targetStart.Y;
    var T0_dot_V = targetStart.X * targetVelocity.X + targetStart.Y * targetVelocity.Y;

    var a = s_sq - v_sq;
    var b = -(2 * s_sq * Delay + 2 * T0_dot_V);
    var c = s_sq * Delay * Delay - T0_sq;

    var disc = b * b - 4 * a * c;
    return (-b + Math.Sqrt(disc)) / (2 * a);
}

// Find T_optimal: minimum T where d_min = collision_threshold (tangent case)
double FindTOptimal(double T_center)
{
    double low = Delay, high = T_center;
    while (high - low > 0.0001)
    {
        var mid = (low + high) / 2;
        var analysis = AnalyzeAimTime(mid);
        if (analysis.HasValue && analysis.Value.minDist <= CollisionThreshold)
            high = mid;
        else
            low = mid;
    }
    return high;
}

Console.WriteLine("=== MINIMUM INTERCEPTION TIME FINDER ===");
Console.WriteLine();
Console.WriteLine($"Target: ({targetStart.X}, {targetStart.Y}) + t*({targetVelocity.X}, {targetVelocity.Y})");
Console.WriteLine($"Projectile: speed={Speed}, delay={Delay}, range={Range}");
Console.WriteLine($"Collision threshold: {CollisionThreshold} (hitbox={HitboxRadius}, half-width={Width/2})");
Console.WriteLine();

var T_center = FindTCenter();
var T_optimal = FindTOptimal(T_center);

Console.WriteLine($"T_center (aim at center):  {T_center:F4}s");
Console.WriteLine($"T_optimal (tangent case):  {T_optimal:F4}s");
Console.WriteLine($"Time saved by hitbox:      {T_center - T_optimal:F4}s");
Console.WriteLine();

var analysis = AnalyzeAimTime(T_optimal);
if (analysis.HasValue)
{
    Console.WriteLine($"At T_optimal:");
    Console.WriteLine($"  Minimum distance to target: {analysis.Value.minDist:F2}");
    Console.WriteLine($"  Actual hit time: {analysis.Value.hitTime:F4}s");
    Console.WriteLine($"  Flight time at hit: {analysis.Value.flightTime:F4}s");
    Console.WriteLine($"  Projectile distance: {Speed * analysis.Value.flightTime:F2}");
}

// Key formulas
Console.WriteLine();
Console.WriteLine("=== FORMULAS ===");
Console.WriteLine();
Console.WriteLine("T_center solves: s·(T - δ) = |T₀ + V·T|");
Console.WriteLine("  Quadratic: (s² - |V|²)T² - 2(s²δ + T₀·V)T + (s²δ² - |T₀|²) = 0");
Console.WriteLine();
Console.WriteLine("T_optimal solves: d_min(T) = collision_threshold");
Console.WriteLine("  where d_min = √(|B|² - (W·B)²/|W|²)");
Console.WriteLine("  W = s·û - V");
Console.WriteLine("  B = T₀ + V·δ");
Console.WriteLine("  û = (T₀ + V·T)/|T₀ + V·T|");
