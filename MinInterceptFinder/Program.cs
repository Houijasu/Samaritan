using MathNet.Spatial.Euclidean;

// Scenario #9: LinearVsMaxRange
const double Delay = 0.25;
const double Speed = 1300.0;
const double Width = 40.0;
const double Range = 1500.0;

var casterPosition = new Point2D(0, 0);
var targetStart = new Point2D(1400, -200);
var targetVelocity = new Vector2D(50, 250);
const double HitboxRadius = 65.0;
const double CollisionThreshold = HitboxRadius + Width / 2.0; // 85

Point2D GetTargetPosition(double t) =>
    new(targetStart.X + targetVelocity.X * t, targetStart.Y + targetVelocity.Y * t);

// Simulation check - does projectile hit when aiming at predicted position?
bool CheckHit(double interceptionTime)
{
    if (interceptionTime < Delay) return false;

    var predictedTargetPos = GetTargetPosition(interceptionTime);
    var aimDistance = casterPosition.DistanceTo(predictedTargetPos);
    if (aimDistance > Range) return false;

    var direction = (predictedTargetPos - casterPosition).Normalize();
    const double dt = 0.0005;

    for (double t = Delay; t <= interceptionTime + 0.5; t += dt)
    {
        var targetPos = GetTargetPosition(t);
        var flightTime = t - Delay;
        var projectileDistance = Speed * flightTime;

        if (projectileDistance > Range) return false;

        var projectilePos = new Point2D(
            casterPosition.X + direction.X * projectileDistance,
            casterPosition.Y + direction.Y * projectileDistance);

        var distance = projectilePos.DistanceTo(targetPos);
        if (distance <= CollisionThreshold)
            return true;
    }

    return false;
}

// NEW FORMULA V1: Using trailing edge + ExactMethod's reduced delay (DOUBLE COUNTING)
double ComputeInterceptionTime_NewFormula_V1()
{
    var targetSpeed = targetVelocity.Length;
    var projectileSpeedSqr = Speed * Speed;

    var cutDistance = Delay * targetSpeed - HitboxRadius;
    var targetDirection = targetVelocity.Normalize();
    var trailingEdgeStart = new Point2D(
        targetStart.X + targetDirection.X * cutDistance,
        targetStart.Y + targetDirection.Y * cutDistance);
    var toTrailingEdge = new Vector2D(
        trailingEdgeStart.X - casterPosition.X,
        trailingEdgeStart.Y - casterPosition.Y);

    var reducedDelay = Delay - HitboxRadius / Speed;

    var quadA = targetVelocity.DotProduct(targetVelocity) - projectileSpeedSqr;
    var quadB = 2.0 * (toTrailingEdge.DotProduct(targetVelocity) + projectileSpeedSqr * reducedDelay);
    var quadC = toTrailingEdge.DotProduct(toTrailingEdge) - projectileSpeedSqr * reducedDelay * reducedDelay;

    var discriminant = quadB * quadB - 4 * quadA * quadC;
    if (discriminant < 0) return double.MaxValue;

    var sqrtDisc = Math.Sqrt(discriminant);
    var t1 = (-quadB + sqrtDisc) / (2 * quadA);
    var t2 = (-quadB - sqrtDisc) / (2 * quadA);

    var minT = double.MaxValue;
    if (t1 >= Delay) minT = Math.Min(minT, t1);
    if (t2 >= Delay) minT = Math.Min(minT, t2);
    return minT;
}

// NEW FORMULA V2: Using raw D with large reduced delay (full collision threshold)
double ComputeInterceptionTime_NewFormula_V2()
{
    var diff = new Vector2D(targetStart.X - casterPosition.X, targetStart.Y - casterPosition.Y);
    var projectileSpeedSqr = Speed * Speed;

    // Use full collision threshold for maximum reduction
    var reducedDelay = Delay - CollisionThreshold / Speed;

    var quadA = targetVelocity.DotProduct(targetVelocity) - projectileSpeedSqr;
    var quadB = 2.0 * (diff.DotProduct(targetVelocity) + projectileSpeedSqr * reducedDelay);
    var quadC = diff.DotProduct(diff) - projectileSpeedSqr * reducedDelay * reducedDelay;

    var discriminant = quadB * quadB - 4 * quadA * quadC;
    if (discriminant < 0) return double.MaxValue;

    var sqrtDisc = Math.Sqrt(discriminant);
    var t1 = (-quadB + sqrtDisc) / (2 * quadA);
    var t2 = (-quadB - sqrtDisc) / (2 * quadA);

    var minT = double.MaxValue;
    if (t1 >= Delay) minT = Math.Min(minT, t1);
    if (t2 >= Delay) minT = Math.Min(minT, t2);
    return minT;
}

// NEW FORMULA V3: Pure trailing edge, no reduced delay terms
double ComputeInterceptionTime_NewFormula_V3()
{
    var targetSpeed = targetVelocity.Length;
    var projectileSpeedSqr = Speed * Speed;

    // Trailing edge with collision threshold
    var cutDistance = Delay * targetSpeed - CollisionThreshold;
    var targetDirection = targetVelocity.Normalize();
    var trailingEdgeStart = new Point2D(
        targetStart.X + targetDirection.X * cutDistance,
        targetStart.Y + targetDirection.Y * cutDistance);
    var toTrailingEdge = new Vector2D(
        trailingEdgeStart.X - casterPosition.X,
        trailingEdgeStart.Y - casterPosition.Y);

    // Standard quadratic (no reduced delay)
    var quadA = targetVelocity.DotProduct(targetVelocity) - projectileSpeedSqr;
    var quadB = 2.0 * toTrailingEdge.DotProduct(targetVelocity);
    var quadC = toTrailingEdge.DotProduct(toTrailingEdge);

    var discriminant = quadB * quadB - 4 * quadA * quadC;
    if (discriminant < 0) return double.MaxValue;

    var sqrtDisc = Math.Sqrt(discriminant);
    var t1 = (-quadB + sqrtDisc) / (2 * quadA);
    var t2 = (-quadB - sqrtDisc) / (2 * quadA);

    var minT = double.MaxValue;
    if (t1 >= Delay) minT = Math.Min(minT, t1);
    if (t2 >= Delay) minT = Math.Min(minT, t2);
    return minT;
}

// T_optimal approximation using sin(θ) formula
double ComputeInterceptionTime_TOptimal()
{
    // First compute T_center
    var s_sq = Speed * Speed;
    var v_sq = targetVelocity.DotProduct(targetVelocity);
    var T0_sq = targetStart.X * targetStart.X + targetStart.Y * targetStart.Y;
    var T0_dot_V = targetStart.X * targetVelocity.X + targetStart.Y * targetVelocity.Y;

    var a = s_sq - v_sq;
    var b = -(2 * s_sq * Delay + 2 * T0_dot_V);
    var c = s_sq * Delay * Delay - T0_sq;
    var disc = b * b - 4 * a * c;
    var T_center = (-b + Math.Sqrt(disc)) / (2 * a);

    // Compute angle at T_center
    var A_center = new Vector2D(targetStart.X + targetVelocity.X * T_center, targetStart.Y + targetVelocity.Y * T_center);
    var u_center = A_center.Normalize();
    var targetSpeed = targetVelocity.Length;
    var cosTheta = (u_center.X * targetVelocity.X + u_center.Y * targetVelocity.Y) / targetSpeed;
    var sinTheta = Math.Sqrt(1 - cosTheta * cosTheta);

    // T_optimal ≈ T_center - r/(|V|·sin(θ))
    return T_center - CollisionThreshold / (targetSpeed * sinTheta);
}

// ExactMethod formula (for comparison)
double ComputeInterceptionTime_ExactMethod()
{
    var diff = new Vector2D(targetStart.X - casterPosition.X, targetStart.Y - casterPosition.Y);
    var sqrSpeed = Speed * Speed;
    var effectiveRadius = HitboxRadius + Width / 2.0;

    var reducedDelay = Delay - effectiveRadius / Speed;

    var a = targetVelocity.DotProduct(targetVelocity) - sqrSpeed;
    var b = 2.0 * (diff.DotProduct(targetVelocity) + sqrSpeed * reducedDelay);
    var c = diff.DotProduct(diff) - sqrSpeed * reducedDelay * reducedDelay;

    var discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return double.MaxValue;

    var sqrtDisc = Math.Sqrt(discriminant);
    var t1 = (-b + sqrtDisc) / (2 * a);
    var t2 = (-b - sqrtDisc) / (2 * a);

    var minT = double.MaxValue;
    if (t1 >= Delay) minT = Math.Min(minT, t1);
    if (t2 >= Delay) minT = Math.Min(minT, t2);

    return minT;
}

// Find true minimum via binary search
double FindTrueMinimum()
{
    double low = Delay, high = 2.0;
    while (high - low > 0.0001)
    {
        var mid = (low + high) / 2;
        if (CheckHit(mid))
            high = mid;
        else
            low = mid;
    }
    return high;
}

Console.WriteLine("=== SCENARIO #9: LinearVsMaxRange ===");
Console.WriteLine();
Console.WriteLine($"Target: ({targetStart.X}, {targetStart.Y}) + t*({targetVelocity.X}, {targetVelocity.Y})");
Console.WriteLine($"Projectile: speed={Speed}, delay={Delay}, range={Range}");
Console.WriteLine($"Hitbox: {HitboxRadius}, Width: {Width}, Collision threshold: {CollisionThreshold}");
Console.WriteLine();

var trueMinimum = FindTrueMinimum();
var v1Time = ComputeInterceptionTime_NewFormula_V1();
var v2Time = ComputeInterceptionTime_NewFormula_V2();
var v3Time = ComputeInterceptionTime_NewFormula_V3();
var tOptimalTime = ComputeInterceptionTime_TOptimal();
var exactMethodTime = ComputeInterceptionTime_ExactMethod();

Console.WriteLine("=== INTERCEPTION TIMES ===");
Console.WriteLine($"  True minimum (binary search):  {trueMinimum:F4}s");
Console.WriteLine($"  V1 (trailing edge + s²d'):     {v1Time:F4}s");
Console.WriteLine($"  V2 (raw D + full threshold):   {v2Time:F4}s");
Console.WriteLine($"  V3 (pure trailing, no s²d'):   {v3Time:F4}s");
Console.WriteLine($"  T_optimal (sin θ formula):     {tOptimalTime:F4}s");
Console.WriteLine($"  ExactMethod (D + s²d'):        {exactMethodTime:F4}s");
Console.WriteLine();

Console.WriteLine("=== VERIFICATION (does it hit?) ===");
Console.WriteLine($"  CheckHit({trueMinimum:F4}) = {CheckHit(trueMinimum)}");
Console.WriteLine($"  CheckHit({v1Time:F4}) V1 = {CheckHit(v1Time)}");
Console.WriteLine($"  CheckHit({v2Time:F4}) V2 = {CheckHit(v2Time)}");
Console.WriteLine($"  CheckHit({v3Time:F4}) V3 = {CheckHit(v3Time)}");
Console.WriteLine($"  CheckHit({tOptimalTime:F4}) T_opt = {CheckHit(tOptimalTime)}");
Console.WriteLine($"  CheckHit({exactMethodTime:F4}) Exact = {CheckHit(exactMethodTime)}");
Console.WriteLine();

Console.WriteLine("=== ERRORS (from true minimum) ===");
Console.WriteLine($"  V1 error:        {(v1Time - trueMinimum) * 1000:F2}ms ({100 * (v1Time - trueMinimum) / trueMinimum:F2}%)");
Console.WriteLine($"  V2 error:        {(v2Time - trueMinimum) * 1000:F2}ms ({100 * (v2Time - trueMinimum) / trueMinimum:F2}%)");
Console.WriteLine($"  V3 error:        {(v3Time - trueMinimum) * 1000:F2}ms ({100 * (v3Time - trueMinimum) / trueMinimum:F2}%)");
Console.WriteLine($"  T_optimal error: {(tOptimalTime - trueMinimum) * 1000:F2}ms ({100 * (tOptimalTime - trueMinimum) / trueMinimum:F2}%)");
Console.WriteLine($"  ExactMethod err: {(exactMethodTime - trueMinimum) * 1000:F2}ms ({100 * (exactMethodTime - trueMinimum) / trueMinimum:F2}%)");
Console.WriteLine();

// Show where each formula aims
Console.WriteLine("=== AIM POINTS ===");
var trueAimPoint = GetTargetPosition(trueMinimum);
var v1AimPoint = GetTargetPosition(v1Time);
var v3AimPoint = GetTargetPosition(v3Time);
var tOptAimPoint = GetTargetPosition(tOptimalTime);
var exactAimPoint = GetTargetPosition(exactMethodTime);
Console.WriteLine($"  True minimum aims at: ({trueAimPoint.X:F1}, {trueAimPoint.Y:F1}) dist={casterPosition.DistanceTo(trueAimPoint):F1}");
Console.WriteLine($"  V1 aims at:           ({v1AimPoint.X:F1}, {v1AimPoint.Y:F1}) dist={casterPosition.DistanceTo(v1AimPoint):F1}");
Console.WriteLine($"  V3 aims at:           ({v3AimPoint.X:F1}, {v3AimPoint.Y:F1}) dist={casterPosition.DistanceTo(v3AimPoint):F1}");
Console.WriteLine($"  T_optimal aims at:    ({tOptAimPoint.X:F1}, {tOptAimPoint.Y:F1}) dist={casterPosition.DistanceTo(tOptAimPoint):F1}");
Console.WriteLine($"  ExactMethod aims at:  ({exactAimPoint.X:F1}, {exactAimPoint.Y:F1}) dist={casterPosition.DistanceTo(exactAimPoint):F1}");
