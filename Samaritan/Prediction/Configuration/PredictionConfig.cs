namespace Samaritan.Prediction.Configuration;

/// <summary>
/// Configuration settings for the prediction engine.
/// </summary>
public sealed record PredictionConfig
{
    /// <summary>
    /// Maximum time horizon for predictions (seconds).
    /// </summary>
    public double MaxPredictionTime { get; init; } = 3.0;

    /// <summary>
    /// Network ping / round-trip latency in milliseconds.
    /// This accounts for the delay between seeing target position and the server
    /// receiving your cast command. The target moves during this time.
    /// </summary>
    public double PingMs { get; init; } = 0;

    /// <summary>
    /// Server tick rate in Hz. League of Legends uses 30 ticks/second.
    /// Used to quantize predictions to server update boundaries.
    /// </summary>
    public double ServerTickRateHz { get; init; } = 30;

    /// <summary>
    /// Extra reaction time buffer in milliseconds to account for human input delay
    /// and processing time between seeing the target and pressing the key.
    /// </summary>
    public double ReactionBufferMs { get; init; } = 0;

    /// <summary>
    /// Gets the total network compensation delay in seconds.
    /// This is added to skillshot delays to account for:
    /// - Full ping (you see old state + command travel time)
    /// - Half a tick uncertainty (server processes at discrete intervals)
    /// - Optional reaction buffer
    /// </summary>
    public double NetworkCompensationDelay =>
        (PingMs + ReactionBufferMs) / 1000.0 + (0.5 / ServerTickRateHz);

    /// <summary>
    /// Gets the server tick duration in seconds.
    /// </summary>
    public double TickDurationSeconds => 1.0 / ServerTickRateHz;

    /// <summary>
    /// Convergence tolerance for numerical solvers (seconds).
    /// </summary>
    public double ConvergenceTolerance { get; init; } = 0.001;

    /// <summary>
    /// Maximum iterations for Newton-Raphson solver.
    /// </summary>
    public int MaxNewtonIterations { get; init; } = 10;

    /// <summary>
    /// Number of binary search iterations for refinement.
    /// </summary>
    public int BinarySearchIterations { get; init; } = 8;

    /// <summary>
    /// Time step for coarse simulation (seconds).
    /// </summary>
    public double CoarseTimeStep { get; init; } = 0.05;

    /// <summary>
    /// Time step for fine refinement (seconds).
    /// </summary>
    public double FineTimeStep { get; init; } = 0.005;

    /// <summary>
    /// Default target hitbox radius (units).
    /// </summary>
    public double DefaultHitboxRadius { get; init; } = 65.0;

    /// <summary>
    /// Minimum confidence threshold for valid predictions.
    /// </summary>
    public double MinConfidence { get; init; } = 0.3;

    /// <summary>
    /// Epsilon for floating-point comparisons.
    /// </summary>
    public double Epsilon { get; init; } = 1e-9;

    /// <summary>
    /// Cache capacity for prediction memoization.
    /// </summary>
    public int CacheCapacity { get; init; } = 256;

    /// <summary>
    /// Cache entry TTL in milliseconds.
    /// </summary>
    public int CacheTtlMs { get; init; } = 100;

    /// <summary>
    /// Default configuration with balanced settings.
    /// </summary>
    public static PredictionConfig Default => new();

    /// <summary>
    /// Fast preset - prioritizes performance over accuracy.
    /// </summary>
    public static PredictionConfig Fast => new()
    {
        CoarseTimeStep = 0.1,
        FineTimeStep = 0.01,
        MaxNewtonIterations = 5,
        BinarySearchIterations = 5
    };

    /// <summary>
    /// Precise preset - prioritizes accuracy over performance.
    /// </summary>
    public static PredictionConfig Precise => new()
    {
        CoarseTimeStep = 0.025,
        FineTimeStep = 0.002,
        MaxNewtonIterations = 15,
        BinarySearchIterations = 12,
        ConvergenceTolerance = 0.0001
    };
}
