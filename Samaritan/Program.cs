using MathNet.Spatial.Euclidean;

using Samaritan;
using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

using Spectre.Console;

AnsiConsole.MarkupLine("[bold blue]Samaritan Skillshot Prediction Engine Demo[/]");
AnsiConsole.WriteLine();

var engine = new PredictionEngine();

// Demo 1: Linear skillshot on stationary target
AnsiConsole.MarkupLine("[yellow]Demo 1: Linear Skillshot (Ezreal Q) vs Stationary Target[/]");
var ezrealQ = new Skillshot.Linear(Delay: 0.25f, Speed: 2000, Width: 60, Range: 1150);
var casterPos = new Point2D(0, 0);

var stationaryTracker = new MovementTracker { HitboxRadius = 65 };
stationaryTracker.Update(new Point2D(800, 0), gameTime: 0);

var result1 = engine.Predict(ezrealQ, casterPos, stationaryTracker);
PrintResult("Ezreal Q", result1);

// Demo 2: Linear skillshot on moving target
AnsiConsole.MarkupLine("[yellow]Demo 2: Linear Skillshot vs Moving Target[/]");
var movingTracker = new MovementTracker { HitboxRadius = 65 };
movingTracker.Update(new Point2D(600, 0), gameTime: 0);
movingTracker.Update(new Point2D(600, 100), gameTime: 0.1); // Moving upward

var result2 = engine.Predict(ezrealQ, casterPos, movingTracker);
PrintResult("Ezreal Q (leading)", result2);

// Demo 3: Circular skillshot
AnsiConsole.MarkupLine("[yellow]Demo 3: Circular Skillshot (Lux E)[/]");
var luxE = new Skillshot.Circular(Delay: 0.25f, Speed: 1200, Radius: 350, Range: 1100);

var result3 = engine.Predict(luxE, casterPos, movingTracker);
PrintResult("Lux E", result3);

// Demo 4: Cone skillshot
AnsiConsole.MarkupLine("[yellow]Demo 4: Cone Skillshot (Annie W)[/]");
var annieW = new Skillshot.Cone(Delay: 0.25f, Angle: 50, Range: 600);
var closeTracker = new MovementTracker { HitboxRadius = 65 };
closeTracker.Update(new Point2D(400, 50), gameTime: 0);

var result4 = engine.Predict(annieW, casterPos, closeTracker);
PrintResult("Annie W", result4);

// Demo 5: Arc skillshot (clockwise vs counter-clockwise)
AnsiConsole.MarkupLine("[yellow]Demo 5: Arc Skillshot (Diana Q) - Counter-Clockwise[/]");
var dianaQ = new Skillshot.Arc(Delay: 0.25f, Speed: 1900, Width: 185, OuterRadius: 900, Angle: 250, Clockwise: false);
var arcTracker = new MovementTracker { HitboxRadius = 65 };
arcTracker.Update(new Point2D(700, 300), gameTime: 0); // Above and to the right

var result5 = engine.Predict(dianaQ, casterPos, arcTracker);
PrintResult("Diana Q (CCW)", result5);

// Demo 6: Vector rectangle skillshot
AnsiConsole.MarkupLine("[yellow]Demo 6: Vector Rectangle Skillshot (Viktor E)[/]");
var viktorE = new Skillshot.VectorRectangle(Delay: 0f, Speed: 1050, Width: 180, MaxLength: 700, Range: 525);
var viktorTracker = new MovementTracker { HitboxRadius = 65 };
viktorTracker.Update(new Point2D(500, 100), gameTime: 0);
viktorTracker.Update(new Point2D(550, 100), gameTime: 0.1);

var result6 = engine.Predict(viktorE, casterPos, viktorTracker);
PrintResult("Viktor E", result6);

// Demo 7: Collision validation
AnsiConsole.MarkupLine("[yellow]Demo 7: Collision Validation[/]");
var targetInPath = new Point2D(400, 0);
var targetOutOfPath = new Point2D(400, 200);

// At 0.5s with 0.25s delay, projectile traveled 0.25s * 2000 = 500 units
var hitInPath = engine.ValidateHit(ezrealQ, casterPos, new Point2D(800, 0), targetInPath, hitboxRadius: 65, timeElapsed: 0.5);
var hitOutPath = engine.ValidateHit(ezrealQ, casterPos, new Point2D(800, 0), targetOutOfPath, hitboxRadius: 65, timeElapsed: 0.5);

AnsiConsole.MarkupLine($"  Target at (400, 0): {(hitInPath ? "[green]HIT[/]" : "[red]MISS[/]")}");
AnsiConsole.MarkupLine($"  Target at (400, 200): {(hitOutPath ? "[green]HIT[/]" : "[red]MISS[/]")}");

AnsiConsole.WriteLine();
AnsiConsole.MarkupLine("[bold green]Demo complete![/]");

static void PrintResult(string name, PredictionResult result)
{
    result.Match(
        hit: h =>
        {
            AnsiConsole.MarkupLine($"  [green]HIT[/] - Time: {h.InterceptionTime:F3}s");
            AnsiConsole.MarkupLine($"    Cast at: ({h.CastPosition.X:F0}, {h.CastPosition.Y:F0})");
            AnsiConsole.MarkupLine($"    Target will be at: ({h.PredictedPosition.X:F0}, {h.PredictedPosition.Y:F0})");
            AnsiConsole.MarkupLine($"    Confidence: {h.Confidence:P0}");
        },
        outOfRange: o =>
        {
            AnsiConsole.MarkupLine($"  [red]OUT OF RANGE[/] - Distance: {o.Distance:F0}, Max: {o.MaxRange:F0}");
        },
        unreachable: u =>
        {
            AnsiConsole.MarkupLine($"  [red]UNREACHABLE[/] - {u.Reason}");
        });
    AnsiConsole.WriteLine();
}
