namespace Samaritan.Benchmarks;

using BenchmarkDotNet.Attributes;

using MathNet.Spatial.Euclidean;

using Samaritan;
using Samaritan.Prediction.Engine;
using Samaritan.Prediction.Movement;
using Samaritan.Prediction.Results;

/// <summary>
/// Computational cost comparison of the prediction techniques on identical
/// inputs (Nidalee Q vs a 350 u/s walker). Caching is disabled so the numbers
/// measure the solvers themselves, not the cache.
/// </summary>
[ShortRunJob]
[MemoryDiagnoser]
public class PredictionBenchmarks
{
    private static readonly Skillshot NidaleeQ = new Skillshot.Linear(
        Delay: 0.25f, Speed: 1300, Width: 40, Range: 1500);

    private static readonly Point2D Caster = new(0, 0);

    private readonly PredictionEngine _engine = new(enableCaching: false);
    private readonly GagongPredictionEngine _gagong = new();

    private MovementState _target = null!;

    [Params("Perpendicular", "Crossing")]
    public string Geometry { get; set; } = "Perpendicular";

    [GlobalSetup]
    public void Setup()
    {
        _target = Geometry == "Perpendicular"
            ? new MovementState.Walking(new Point2D(600, -200), new Vector2D(0, 350), null)
            : new MovementState.Walking(new Point2D(1000, 0), new Vector2D(-120, 330), null);
    }

    [Benchmark(Baseline = true)]
    public PredictionResult RearGraze() =>
        _engine.PredictFromState(NidaleeQ, Caster, _target, 65, ProjectileAimMode.RearGraze);

    [Benchmark]
    public PredictionResult NearestRear() =>
        _engine.PredictFromState(NidaleeQ, Caster, _target, 65, ProjectileAimMode.NearestRear);

    [Benchmark]
    public PredictionResult Optimal() =>
        _engine.PredictFromState(NidaleeQ, Caster, _target, 65, ProjectileAimMode.Optimal);

    [Benchmark]
    public PredictionResult Gagong() =>
        _gagong.PredictFromState(NidaleeQ, Caster, _target, 65);
}
