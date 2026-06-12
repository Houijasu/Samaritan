namespace Samaritan.Simulation.Rendering;

using FontStashSharp;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

using Samaritan.Prediction.Results;
using Samaritan.Simulation.Core;
using Samaritan.Simulation.Scenarios;

/// <summary>
/// Renders the HUD overlay with metrics and controls.
/// </summary>
public class HudRenderer
{
    private readonly GraphicsDevice _device;
    private readonly SpriteBatch _spriteBatch;
    private readonly Texture2D _pixel;
    private readonly FontSystem _fontSystem;
    private readonly SpriteFontBase _font;
    private readonly SpriteFontBase _fontSmall;
    private readonly SpriteFontBase _fontLarge;

    private static readonly Color BackgroundColor = new(0, 0, 0, 200);
    private static readonly Color TextColor = new(220, 220, 220);
    private static readonly Color HitColor = new(80, 220, 80);
    private static readonly Color MissColor = new(220, 80, 80);
    private static readonly Color WarningColor = new(220, 200, 50);
    private static readonly Color DimColor = new(140, 140, 140);

    public HudRenderer(GraphicsDevice device, SpriteBatch spriteBatch)
    {
        _device = device;
        _spriteBatch = spriteBatch;

        // Create 1x1 white texture for drawing rectangles
        _pixel = new Texture2D(device, 1, 1);
        _pixel.SetData([Color.White]);

        // Create font system with embedded default font
        _fontSystem = new FontSystem();

        // Use embedded default font (DejaVu Sans Mono)
        _fontSystem.AddFont(GetEmbeddedFont());

        _fontSmall = _fontSystem.GetFont(14);
        _font = _fontSystem.GetFont(16);
        _fontLarge = _fontSystem.GetFont(20);
    }

    private static byte[] GetEmbeddedFont()
    {
        // Try to load a monospace font from system fonts
        var fontPaths = new[]
        {
            // Arch Linux / modern distros
            "/usr/share/fonts/Adwaita/AdwaitaMono-Regular.ttf",
            "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
            "/usr/share/fonts/dejavu/DejaVuSansMono.ttf",
            // Ubuntu/Debian
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
            // Fedora
            "/usr/share/fonts/liberation-mono/LiberationMono-Regular.ttf",
            // Windows
            "C:/Windows/Fonts/consola.ttf",
            "C:/Windows/Fonts/cour.ttf",
            // macOS
            "/System/Library/Fonts/Monaco.ttf",
            "/System/Library/Fonts/Menlo.ttc"
        };

        foreach (var path in fontPaths)
        {
            if (File.Exists(path))
            {
                return File.ReadAllBytes(path);
            }
        }

        throw new FileNotFoundException("Could not find a suitable system font. Searched: " + string.Join(", ", fontPaths));
    }

    public void Draw(
        Scenario scenario,
        SimulationState state,
        int scenarioIndex,
        int totalScenarios,
        double speed,
        bool isPaused,
        PredictionMethod method)
    {
        _spriteBatch.Begin();

        // Top-left panel: Scenario info
        DrawPanel(10, 10, 350, 160);
        DrawText($"Scenario {scenarioIndex + 1}/{totalScenarios}", 20, 18, TextColor, _fontLarge);
        DrawText(scenario.Name, 20, 48, WarningColor, _font);
        DrawText($"Skillshot: {GetSkillshotTypeName(scenario.Skillshot)}", 20, 73, TextColor, _font);
        DrawText($"Target Hitbox: {scenario.HitboxRadius:F0} units", 20, 98, TextColor, _font);

        var (methodLabel, methodColor) = method switch
        {
            PredictionMethod.Nearest => ("NEAREST (tangent)", Color.Cyan),
            PredictionMethod.Optimal => ("OPTIMAL (fast rear)", Color.Orange),
            PredictionMethod.Gagong => ("GAGONG (lua port)", Color.Violet),
            _ => ("AFTER (rear graze)", HitColor)
        };
        DrawText($"Method: {methodLabel}", 20, 128, methodColor, _font);

        // Top-right panel: Playback controls
        var rightX = _device.Viewport.Width - 300;
        DrawPanel(rightX, 10, 290, 130);
        DrawText("Controls", rightX + 10, 18, TextColor, _fontLarge);

        var statusColor = isPaused ? WarningColor : HitColor;
        var statusText = isPaused ? "PAUSED" : "RUNNING";
        DrawText($"Speed: {speed:F2}x  [{statusText}]", rightX + 10, 48, statusColor, _font);
        DrawText("Space: Play/Pause   R: Reset", rightX + 10, 73, DimColor, _fontSmall);
        DrawText("Left/Right: Scenarios   +/-: Speed", rightX + 10, 93, DimColor, _fontSmall);
        DrawText("M: Method (Before/After)   Esc: Exit", rightX + 10, 113, DimColor, _fontSmall);

        // Bottom panel: Simulation metrics
        var bottomY = _device.Viewport.Height - 260;
        DrawPanel(10, bottomY, 500, 250);
        DrawText("Simulation Results", 20, bottomY + 8, TextColor, _fontLarge);
        DrawText($"Time: {state.Time:F3}s   Phase: {state.Phase}", 20, bottomY + 38, TextColor, _font);

        // Prediction result
        var (predictionText, predictionColor) = GetPredictionDisplay(state);
        DrawText(predictionText, 20, bottomY + 63, predictionColor, _font);

        // Exact comparison
        if (state.ExactPredictedTime.HasValue)
        {
            var diff = state.Prediction is PredictionResult.Hit h ? h.InterceptionTime - state.ExactPredictedTime.Value : 0;
            var compText = $"Exact Method: {state.ExactPredictedTime:F3}s (Diff: {diff:F3}s)";
            DrawText(compText, 20, bottomY + 88, Color.Cyan, _font);
        }

        // Angle geometry
        if (state.CosTheta.HasValue && state.SinTheta.HasValue)
        {
            var angleText = $"cosθ: {state.CosTheta:F4}   sinθ: {state.SinTheta:F4}";
            DrawText(angleText, 20, bottomY + 113, DimColor, _font);
        }

        // Actual result
        if (state.Phase == SimulationPhase.Complete)
        {
            var actualText = state.ActualHitTime.HasValue
                ? $"Actual: HIT at {state.ActualHitTime:F3}s"
                : "Actual: MISS - projectile did not hit target";
            var actualColor = state.ActualHitTime.HasValue ? HitColor : MissColor;
            DrawText(actualText, 20, bottomY + 138, actualColor, _font);

            // Error metrics
            if (state.PositionError.HasValue)
            {
                var errorColor = state.PositionError.Value < 20 ? HitColor :
                                state.PositionError.Value < 50 ? WarningColor : MissColor;
                DrawText($"Position Error: {state.PositionError:F1} units   Time Error: {state.TimeError:F3}s",
                    20, bottomY + 163, errorColor, _font);
            }
        }
        else
        {
            DrawText($"Target Position: ({state.TargetPosition.X:F0}, {state.TargetPosition.Y:F0})",
                20, bottomY + 138, TextColor, _font);

            DrawText($"Caster Position: ({scenario.CasterPosition.X:F0}, {scenario.CasterPosition.Y:F0})",
                20, bottomY + 163, TextColor, _font);

            var velStr = $"Velocity: ({state.TargetVelocity.X:F0}, {state.TargetVelocity.Y:F0})";
            DrawText(velStr, 20, bottomY + 188, TextColor, _font);
        }

        // Graze margin: how far the simulated flight is from the tangency boundary
        if (state.GrazeGap.HasValue && state.GrazeRadius.HasValue)
        {
            var margin = state.GrazeRadius.Value - state.GrazeGap.Value;
            var verdict = margin >= 0 ? $"HIT by {margin:F1}" : $"MISS by {-margin:F1}";
            var grazeColor = margin >= 0 ? HitColor : MissColor;
            var angleText = state.ApproachAngleDegrees.HasValue
                ? $"   Ray angle: {state.ApproachAngleDegrees:F1} deg"
                : "";
            DrawText($"Graze: {state.GrazeGap:F1} / R {state.GrazeRadius:F0} ({verdict}){angleText}",
                20, bottomY + 213, grazeColor, _font);
        }

        // Legend panel (bottom right)
        var legendX = _device.Viewport.Width - 220;
        var legendY = _device.Viewport.Height - 140;
        DrawPanel(legendX, legendY, 210, 130);
        DrawText("Legend", legendX + 10, legendY + 8, TextColor, _fontLarge);
        DrawText("Blue: Caster", legendX + 10, legendY + 35, new Color(80, 130, 220), _fontSmall);
        DrawText("Red: Target", legendX + 10, legendY + 53, new Color(220, 80, 80), _fontSmall);
        DrawText("Yellow: Predicted", legendX + 10, legendY + 71, new Color(220, 200, 50), _fontSmall);
        DrawText("Cyan: Exact Method", legendX + 10, legendY + 89, Color.Cyan, _fontSmall);
        DrawText("Green: Actual Hit", legendX + 10, legendY + 107, new Color(80, 220, 80), _fontSmall);

        _spriteBatch.End();
    }

    private (string text, Color color) GetPredictionDisplay(SimulationState state)
    {
        if (state.Prediction is null)
            return ("No prediction computed", DimColor);

        return state.Prediction.Match(
            hit: h => ($"Prediction: HIT at {h.InterceptionTime:F3}s (Confidence: {h.Confidence:P0})", HitColor),
            outOfRange: o => ($"Prediction: OUT OF RANGE ({o.Distance:F0} > {o.MaxRange:F0})", MissColor),
            unreachable: u => ($"Prediction: UNREACHABLE - {u.Reason}", MissColor));
    }

    private void DrawPanel(int x, int y, int width, int height)
    {
        _spriteBatch.Draw(_pixel, new Rectangle(x, y, width, height), BackgroundColor);
        // Border
        _spriteBatch.Draw(_pixel, new Rectangle(x, y, width, 1), new Color(60, 60, 70));
        _spriteBatch.Draw(_pixel, new Rectangle(x, y + height - 1, width, 1), new Color(60, 60, 70));
        _spriteBatch.Draw(_pixel, new Rectangle(x, y, 1, height), new Color(60, 60, 70));
        _spriteBatch.Draw(_pixel, new Rectangle(x + width - 1, y, 1, height), new Color(60, 60, 70));
    }

    private void DrawText(string text, int x, int y, Color color, SpriteFontBase font)
    {
        font.DrawText(_spriteBatch, text, new Vector2(x, y), color);
    }

    private static string GetSkillshotTypeName(Skillshot skillshot)
    {
        return skillshot.Match(
            linear: l => $"Linear (Speed: {l.Speed}, Width: {l.Width})",
            circular: c => $"Circular (Radius: {c.Radius})",
            cone: c => $"Cone (Angle: {c.Angle}°)",
            arc: a => $"Arc (Radius: {a.OuterRadius})",
            rectangle: r => $"Rectangle ({r.Width}x{r.Length})",
            vectorRectangle: v => $"Vector Rect ({v.Width}x{v.MaxLength})");
    }
}
