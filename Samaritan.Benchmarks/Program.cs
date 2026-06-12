using BenchmarkDotNet.Running;

using Samaritan.Benchmarks;

BenchmarkSwitcher.FromAssembly(typeof(PredictionBenchmarks).Assembly).Run(args);
