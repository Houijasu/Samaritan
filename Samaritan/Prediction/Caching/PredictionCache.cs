namespace Samaritan.Prediction.Caching;

using System.Collections.Concurrent;

using Samaritan.Prediction.Results;

/// <summary>
/// LRU cache for prediction results with TTL expiration.
/// </summary>
public sealed class PredictionCache
{
    private readonly ConcurrentDictionary<string, CacheEntry> _cache;
    private readonly int _capacity;
    private readonly int _ttlMs;
    private long _accessCounter;

    /// <summary>
    /// Creates a prediction cache.
    /// </summary>
    /// <param name="capacity">Maximum number of entries.</param>
    /// <param name="ttlMs">Time-to-live in milliseconds.</param>
    public PredictionCache(int capacity = 256, int ttlMs = 100)
    {
        _cache = new ConcurrentDictionary<string, CacheEntry>();
        _capacity = capacity;
        _ttlMs = ttlMs;
    }

    /// <summary>
    /// Tries to get a cached result.
    /// </summary>
    public bool TryGet(string key, out PredictionResult result)
    {
        if (_cache.TryGetValue(key, out var entry))
        {
            // Check TTL
            if (Environment.TickCount64 - entry.Timestamp <= _ttlMs)
            {
                entry.LastAccess = Interlocked.Increment(ref _accessCounter);
                result = entry.Result;
                return true;
            }

            // Expired, remove
            _cache.TryRemove(key, out _);
        }

        result = default!;
        return false;
    }

    /// <summary>
    /// Adds or updates a cached result.
    /// </summary>
    public void Set(string key, PredictionResult result)
    {
        // Evict if at capacity
        if (_cache.Count >= _capacity)
        {
            EvictOldest();
        }

        var entry = new CacheEntry
        {
            Result = result,
            Timestamp = Environment.TickCount64,
            LastAccess = Interlocked.Increment(ref _accessCounter)
        };

        _cache.AddOrUpdate(key, entry, (_, _) => entry);
    }

    /// <summary>
    /// Clears all cached entries.
    /// </summary>
    public void Clear()
    {
        _cache.Clear();
    }

    /// <summary>
    /// Gets the current number of cached entries.
    /// </summary>
    public int Count => _cache.Count;

    private void EvictOldest()
    {
        // Find and remove the least recently used entry
        string? oldestKey = null;
        long oldestAccess = long.MaxValue;

        foreach (var kvp in _cache)
        {
            if (kvp.Value.LastAccess < oldestAccess)
            {
                oldestAccess = kvp.Value.LastAccess;
                oldestKey = kvp.Key;
            }
        }

        if (oldestKey is not null)
        {
            _cache.TryRemove(oldestKey, out _);
        }
    }

    private sealed class CacheEntry
    {
        public required PredictionResult Result { get; init; }
        public required long Timestamp { get; init; }
        public long LastAccess { get; set; }
    }
}
