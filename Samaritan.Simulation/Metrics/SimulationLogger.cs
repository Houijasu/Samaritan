namespace Samaritan.Simulation.Metrics;

using System.Text;

using MathNet.Spatial.Euclidean;

using Samaritan.Prediction.Results;

public class SimulationLogger
{
    private readonly string _filePath;
    private readonly object _lock = new();

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

    public void LogComparison(string scenarioName, PredictionResult current, PredictionResult exact)
    {
        var sb = new StringBuilder();
        sb.Append($"{DateTime.Now:HH:mm:ss},");
        sb.Append($"\"{scenarioName}\",");

        // Current Method
        if (current is PredictionResult.Hit h1)
        {
            sb.Append($"{h1.InterceptionTime:F4},");
            sb.Append($"{h1.PredictedPosition.X:F2},"); // Using this slot for X to keep CSV simple? 
            // Wait, I defined specific columns. Let's stick to them.
            // Current_Time
        }
        else
        {
            sb.Append("NaN,");
        }

        // Exact Method
        if (exact is PredictionResult.Hit h2)
        {
            sb.Append($"{h2.InterceptionTime:F4},");
        }
        else
        {
            sb.Append("NaN,");
        }

        // Diff
        if (current is PredictionResult.Hit hit1 && exact is PredictionResult.Hit hit2)
        {
            sb.Append($"{(hit1.InterceptionTime - hit2.InterceptionTime):F4},");

            // Positions
            sb.Append($"{hit1.PredictedPosition.X:F2},");
            sb.Append($"{hit1.PredictedPosition.Y:F2},");
            sb.Append($"{hit2.PredictedPosition.X:F2},");
            sb.Append($"{hit2.PredictedPosition.Y:F2},");
        }
        else
        {
            sb.Append("NaN,NaN,NaN,NaN,NaN,");
        }

        // Notes (Status)
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
