namespace Samaritan.Simulation.Rendering;

using MathNet.Spatial.Euclidean;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

using Samaritan.Simulation.Core;
using Samaritan.Simulation.Scenarios;
using Samaritan.Simulation;

/// <summary>
/// Renders simulation entities (caster, target, projectile, skillshot shapes).
/// </summary>
public class EntityRenderer
{
    private readonly GraphicsDevice _device;
    private readonly BasicEffect _effect;
    private readonly float _scale;

    // Colors
    private static readonly Color CasterColor = new(80, 130, 220);
    private static readonly Color TargetColor = new(220, 80, 80);
    private static readonly Color ProjectileColor = new(255, 255, 255);
    private static readonly Color PredictedColor = new(220, 200, 50);
    private static readonly Color ActualHitColor = new(80, 220, 80);
    private static readonly Color ErrorLineColor = new(220, 80, 220);
    private static readonly Color SkillshotColor = new(100, 100, 180, 60);
    private static readonly Color HandleColor = new(255, 255, 100);
    private static readonly Color HandleActiveColor = new(255, 200, 50);
    private static readonly Color DirectionHandleColor = new(100, 255, 100);

    public EntityRenderer(GraphicsDevice device, float scale, Vector2 offset)
    {
        _device = device;
        _scale = scale;

        _effect = new BasicEffect(device)
        {
            VertexColorEnabled = true,
            View = Matrix.Identity,
            World = Matrix.Identity
        };

        UpdateProjection();
    }

    private void UpdateProjection()
    {
        var viewport = _device.Viewport;
        _effect.Projection = Matrix.CreateOrthographicOffCenter(0, viewport.Width, viewport.Height, 0, 0, 1);
    }

    public void Draw(SimulationState state, Scenario scenario)
    {
        UpdateProjection();

        // Draw skillshot shape (range indicator)
        DrawSkillshotRange(scenario);

        // Draw prediction markers
        if (state.PredictedTargetPosition.HasValue)
        {
            DrawCircle(state.PredictedTargetPosition.Value, scenario.HitboxRadius, PredictedColor, dashed: true);
        }

        // Draw exact prediction (comparison)
        if (state.ExactPredictedPosition.HasValue)
        {
            DrawCircle(state.ExactPredictedPosition.Value, scenario.HitboxRadius, Color.Cyan, dashed: true);
        }

        // Draw actual hit position
        if (state.ActualHitPosition.HasValue)
        {
            DrawCircle(state.ActualHitPosition.Value, scenario.HitboxRadius, ActualHitColor);

            // Draw error line
            if (state.PredictedTargetPosition.HasValue)
            {
                DrawLine(state.PredictedTargetPosition.Value, state.ActualHitPosition.Value, ErrorLineColor);
            }
        }

        // Draw caster
        DrawFilledCircle(scenario.CasterPosition, 30, CasterColor);

        // Draw target
        DrawFilledCircle(state.TargetPosition, 25, TargetColor);
        DrawCircle(state.TargetPosition, scenario.HitboxRadius, new Color(TargetColor, 100));

        // Draw projectile with actual skillshot width
        if (state.ProjectilePosition.HasValue)
        {
            DrawProjectile(state, scenario);
        }
    }

    /// <summary>
    /// Draws interactive drag handles for editing caster, target, and direction.
    /// </summary>
    public void DrawDragHandles(Point2D casterPos, Point2D targetPos, Point2D directionPos, DragMode activeMode)
    {
        // Draw direction line from target to direction handle
        DrawLine(targetPos, directionPos, new Color(DirectionHandleColor, 150));

        // Draw caster handle - large circle to indicate draggable
        var casterColor = activeMode == DragMode.Caster ? HandleActiveColor : HandleColor;
        DrawCircle(casterPos, 50, casterColor);
        DrawCircle(casterPos, 45, casterColor);
        DrawFilledCircle(casterPos, 15, casterColor);

        // Draw target handle - large circle to indicate draggable
        var targetColor = activeMode == DragMode.Target ? HandleActiveColor : HandleColor;
        DrawCircle(targetPos, 50, targetColor);
        DrawCircle(targetPos, 45, targetColor);
        DrawFilledCircle(targetPos, 15, targetColor);

        // Draw direction handle (arrow tip) - larger
        var dirColor = activeMode == DragMode.Direction ? HandleActiveColor : DirectionHandleColor;
        DrawFilledCircle(directionPos, 20, dirColor);
        DrawCircle(directionPos, 35, dirColor);
        DrawCircle(directionPos, 30, dirColor);

        // Draw arrow head at direction handle
        var dir = (directionPos - targetPos);
        if (dir.Length > 10)
        {
            var normalized = dir.Normalize();
            var perpendicular = new Vector2D(-normalized.Y, normalized.X);
            var arrowBase = directionPos - normalized.ScaleBy(25);
            var arrowLeft = arrowBase + perpendicular.ScaleBy(15);
            var arrowRight = arrowBase - perpendicular.ScaleBy(15);
            DrawLine(directionPos, arrowLeft, dirColor);
            DrawLine(directionPos, arrowRight, dirColor);
        }
    }

    private void DrawProjectile(SimulationState state, Scenario scenario)
    {
        if (!state.ProjectilePosition.HasValue) return;

        var projectilePos = state.ProjectilePosition.Value;

        // Get skillshot width for proper visualization
        var skillshotWidth = GetSkillshotWidth(scenario.Skillshot);

        if (skillshotWidth > 0 && state.CastPosition.HasValue)
        {
            // Draw the projectile as a rectangle showing actual skillshot width
            var direction = (state.CastPosition.Value - scenario.CasterPosition).Normalize();
            var perpendicular = new Vector2D(-direction.Y, direction.X);
            var halfWidth = skillshotWidth / 2.0;

            // Draw skillshot body as a filled rectangle from caster to current position
            DrawSkillshotBody(scenario.CasterPosition, projectilePos, halfWidth, direction, perpendicular);

            // Draw the skillshot "head" line at current position
            var p1 = projectilePos + perpendicular.ScaleBy(halfWidth);
            var p2 = projectilePos + perpendicular.ScaleBy(-halfWidth);
            DrawLine(p1, p2, ProjectileColor);

            // Draw small circle at center for visibility
            DrawFilledCircle(projectilePos, 6, ProjectileColor);
        }
        else
        {
            // Fallback for non-linear skillshots
            DrawFilledCircle(projectilePos, 15, ProjectileColor);
        }
    }

    private void DrawSkillshotBody(Point2D start, Point2D end, double halfWidth, Vector2D _, Vector2D perpendicular)
    {
        // Draw the skillshot as a semi-transparent rectangle
        var color = new Color(ProjectileColor, 80);

        var p1 = start + perpendicular.ScaleBy(halfWidth);
        var p2 = start + perpendicular.ScaleBy(-halfWidth);
        var p3 = end + perpendicular.ScaleBy(-halfWidth);
        var p4 = end + perpendicular.ScaleBy(halfWidth);

        var s1 = WorldToScreen(p1);
        var s2 = WorldToScreen(p2);
        var s3 = WorldToScreen(p3);
        var s4 = WorldToScreen(p4);

        // Draw as two triangles
        var vertices = new[]
        {
            new VertexPositionColor(new Vector3(s1.X, s1.Y, 0), color),
            new VertexPositionColor(new Vector3(s2.X, s2.Y, 0), color),
            new VertexPositionColor(new Vector3(s3.X, s3.Y, 0), color),
            new VertexPositionColor(new Vector3(s1.X, s1.Y, 0), color),
            new VertexPositionColor(new Vector3(s3.X, s3.Y, 0), color),
            new VertexPositionColor(new Vector3(s4.X, s4.Y, 0), color),
        };

        foreach (var pass in _effect.CurrentTechnique.Passes)
        {
            pass.Apply();
            _device.DrawUserPrimitives(PrimitiveType.TriangleList, vertices, 0, 2);
        }

        // Draw outline
        var outlineColor = new Color(ProjectileColor, 150);
        DrawLine(p1, p2, outlineColor);
        DrawLine(p2, p3, outlineColor);
        DrawLine(p3, p4, outlineColor);
        DrawLine(p4, p1, outlineColor);
    }

    private static double GetSkillshotWidth(Skillshot skillshot)
    {
        return skillshot.Match(
            linear: l => l.Width,
            circular: c => c.Radius * 2,
            cone: _ => 0,
            arc: a => a.Width,
            rectangle: r => r.Width,
            vectorRectangle: v => v.Width);
    }

    private void DrawSkillshotRange(Scenario scenario)
    {
        scenario.Skillshot.Match(
            linear: l => DrawLinearRange(scenario.CasterPosition, l),
            circular: c => DrawCircle(scenario.CasterPosition, c.Range, SkillshotColor),
            cone: c => DrawConeRange(scenario.CasterPosition, c),
            arc: a => DrawArcRange(scenario.CasterPosition, a),
            rectangle: r => DrawRectangleRange(scenario.CasterPosition, r),
            vectorRectangle: v => DrawCircle(scenario.CasterPosition, v.Range + v.MaxLength, SkillshotColor));
    }

    private void DrawLinearRange(Point2D origin, Skillshot.Linear linear)
    {
        DrawCircle(origin, linear.Range, SkillshotColor);
    }

    private void DrawConeRange(Point2D origin, Skillshot.Cone cone)
    {
        var vertices = new List<VertexPositionColor>();
        var screenOrigin = WorldToScreen(origin);
        var color = SkillshotColor;

        // Draw cone arc
        const int segments = 32;
        var halfAngle = cone.Angle / 2.0 * Math.PI / 180.0;

        for (var i = 0; i <= segments; i++)
        {
            var angle = -halfAngle + (2 * halfAngle * i / segments);
            var x = origin.X + Math.Cos(angle) * cone.Range;
            var y = origin.Y + Math.Sin(angle) * cone.Range;
            var screenPos = WorldToScreen(new Point2D(x, y));

            vertices.Add(new VertexPositionColor(new Vector3(screenOrigin.X, screenOrigin.Y, 0), color));
            vertices.Add(new VertexPositionColor(new Vector3(screenPos.X, screenPos.Y, 0), color));
        }

        if (vertices.Count >= 2)
        {
            DrawLines(vertices.ToArray());
        }
    }

    private void DrawArcRange(Point2D origin, Skillshot.Arc arc)
    {
        // Draw outer radius
        DrawCircle(origin, arc.OuterRadius, SkillshotColor);
        // Draw inner radius
        DrawCircle(origin, arc.OuterRadius - arc.Width, new Color(SkillshotColor, 30));
    }

    private void DrawRectangleRange(Point2D origin, Skillshot.Rectangle rect)
    {
        DrawCircle(origin, rect.Range, SkillshotColor);
    }

    private void DrawCircle(Point2D center, double radius, Color color, bool dashed = false)
    {
        const int segments = 64;
        var vertices = new List<VertexPositionColor>();

        for (var i = 0; i < segments; i++)
        {
            if (dashed && i % 4 >= 2) continue; // Skip every other pair for dashed effect

            var angle1 = 2 * Math.PI * i / segments;
            var angle2 = 2 * Math.PI * (i + 1) / segments;

            var p1 = new Point2D(
                center.X + Math.Cos(angle1) * radius,
                center.Y + Math.Sin(angle1) * radius);
            var p2 = new Point2D(
                center.X + Math.Cos(angle2) * radius,
                center.Y + Math.Sin(angle2) * radius);

            var s1 = WorldToScreen(p1);
            var s2 = WorldToScreen(p2);

            vertices.Add(new VertexPositionColor(new Vector3(s1.X, s1.Y, 0), color));
            vertices.Add(new VertexPositionColor(new Vector3(s2.X, s2.Y, 0), color));
        }

        if (vertices.Count >= 2)
        {
            DrawLines(vertices.ToArray());
        }
    }

    private void DrawFilledCircle(Point2D center, double radius, Color color)
    {
        const int segments = 32;
        var screenCenter = WorldToScreen(center);
        var screenRadius = (float)(radius * _scale);

        var vertices = new VertexPositionColor[segments * 3];

        for (var i = 0; i < segments; i++)
        {
            var angle1 = 2 * Math.PI * i / segments;
            var angle2 = 2 * Math.PI * (i + 1) / segments;

            vertices[i * 3] = new VertexPositionColor(
                new Vector3(screenCenter.X, screenCenter.Y, 0), color);
            vertices[i * 3 + 1] = new VertexPositionColor(
                new Vector3(
                    screenCenter.X + (float)Math.Cos(angle1) * screenRadius,
                    screenCenter.Y + (float)Math.Sin(angle1) * screenRadius, 0), color);
            vertices[i * 3 + 2] = new VertexPositionColor(
                new Vector3(
                    screenCenter.X + (float)Math.Cos(angle2) * screenRadius,
                    screenCenter.Y + (float)Math.Sin(angle2) * screenRadius, 0), color);
        }

        foreach (var pass in _effect.CurrentTechnique.Passes)
        {
            pass.Apply();
            _device.DrawUserPrimitives(PrimitiveType.TriangleList, vertices, 0, segments);
        }
    }

    private void DrawLine(Point2D p1, Point2D p2, Color color)
    {
        var s1 = WorldToScreen(p1);
        var s2 = WorldToScreen(p2);

        var vertices = new[]
        {
            new VertexPositionColor(new Vector3(s1.X, s1.Y, 0), color),
            new VertexPositionColor(new Vector3(s2.X, s2.Y, 0), color)
        };

        DrawLines(vertices);
    }

    private void DrawLines(VertexPositionColor[] vertices)
    {
        if (vertices.Length < 2) return;

        foreach (var pass in _effect.CurrentTechnique.Passes)
        {
            pass.Apply();
            _device.DrawUserPrimitives(PrimitiveType.LineList, vertices, 0, vertices.Length / 2);
        }
    }

    private Vector2 WorldToScreen(Point2D world)
    {
        var viewport = _device.Viewport;
        var offsetX = viewport.Width / 2f;
        var offsetY = viewport.Height / 2f;

        return new Vector2(
            (float)(world.X * _scale + offsetX),
            (float)(world.Y * _scale + offsetY));
    }
}
