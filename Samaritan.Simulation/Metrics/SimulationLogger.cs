namespace Samaritan.Simulation.Metrics;

using System.Text;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Results;

/// <summary>
/// Logs simulation results to CSV for analysis and comparison.
/// </summary>
public class SimulationLogger
{
    private readonly string _filePath;
    private readonly object _lock = new();

    /// <summary>
    /// Creates a new simulation logger.
    /// </summary>
    /// <param name="fileName">Output CSV filename (default: simulation_results.csv).</param>
    public SimulationLogger(string fileName = "simulation_results.csv")
    {
        _filePath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, fileName);
        Initialize();
    }

    private void Initialize()
    {
        if (!File.Exists(_filePath))
        {
            var header = "Timestamp,Scenario,Current_Time,Exact_Time,Diff_Time,Current_Pos_X,Current_Pos_Y,Exact_Pos_X,Exact_Pos_Y,Notes\n";
            File.WriteAllText(_filePath, header);
        }
    }

    /// <summary>
    /// Logs a comparison between current prediction method and exact analytical solution.
    /// </summary>
    /// <param name="scenarioName">Name of the scenario being tested.</param>
    /// <param name="current">Result from the current prediction method.</param>
    /// <param name="exact">Result from the exact analytical method.</param>
    public void LogComparison(string scenarioName, PredictionResult current, PredictionResult exact)
    {
        var sb = new StringBuilder();
        sb.Append($"{DateTime.Now:HH:mm:ss},");
        sb.Append($"\"{scenarioName}\",");

        // Current method time
        if (current is PredictionResult.Hit h1)
        {
            sb.Append($"{h1.InterceptionTime:F4},");
        }
        else
        {
            sb.Append("NaN,");
        }

        // Exact method time
        if (exact is PredictionResult.Hit h2)
        {
            sb.Append($"{h2.InterceptionTime:F4},");
        }
        else
        {
            sb.Append("NaN,");
        }

        // Time difference and position data
        if (current is PredictionResult.Hit hit1 && exact is PredictionResult.Hit hit2)
        {
            sb.Append($"{(hit1.InterceptionTime - hit2.InterceptionTime):F4},");
            sb.Append($"{hit1.PredictedPosition.X:F2},");
            sb.Append($"{hit1.PredictedPosition.Y:F2},");
            sb.Append($"{hit2.PredictedPosition.X:F2},");
            sb.Append($"{hit2.PredictedPosition.Y:F2},");
        }
        else
        {
            sb.Append("NaN,NaN,NaN,NaN,NaN,");
        }

        // Notes field for non-hit prediction results
        var status = "";
        if (current is not PredictionResult.Hit) status += $"Current:{current.GetType().Name} ";
        if (exact is not PredictionResult.Hit) status += $"Exact:{exact.GetType().Name}";
        sb.Append($"\"{status.Trim()}\"");

        sb.Append('\n');

        lock (_lock)
        {
            File.AppendAllText(_filePath, sb.ToString());
        }
    }
}
