// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;

namespace PolygonClipper.Benchmarks;

/// <summary>
/// <para>
/// Micro benchmarks for the primitives the sweep line calls most often, so that a regression in
/// one of them cannot hide inside an end to end measurement.
/// </para>
/// <para>
/// Every benchmark loops over a prepared batch rather than measuring a single call, because a
/// single <c>SignedArea</c> is far below BenchmarkDotNet's resolution. Compare the numbers
/// against each other across runs, not as an absolute per-call cost.
/// </para>
/// <para>
/// <c>StatusLine</c> and <c>SegmentComparer</c> in its status-line role are deliberately absent.
/// Their cost depends on the state the sweep has built up at that point, and a synthetic state
/// would measure something the algorithm never actually does. <see cref="ScalingBenchmarks"/>
/// covers them end to end instead.
/// </para>
/// </summary>
[MemoryDiagnoser]
public class PrimitiveBenchmarks
{
    private const int BatchSize = 1024;

    private Vertex[] triples = null!;
    private Segment[] segmentPairs = null!;
    private SweepEvent[] eventPairs = null!;

    [GlobalSetup]
    public void Setup()
    {
        // A fixed seed keeps the batches identical between runs so results stay comparable.
        Random random = new(982451653);

        this.triples = new Vertex[BatchSize * 3];
        for (int i = 0; i < this.triples.Length; i++)
        {
            this.triples[i] = new Vertex(random.NextDouble() * 100D, random.NextDouble() * 100D);
        }

        // Half the pairs are built to cross, half to miss, so neither branch dominates.
        this.segmentPairs = new Segment[BatchSize * 2];
        for (int i = 0; i < BatchSize; i++)
        {
            double x = random.NextDouble() * 100D;
            double y = random.NextDouble() * 100D;

            this.segmentPairs[i * 2] = new Segment(new Vertex(x, y), new Vertex(x + 10D, y + 10D));
            this.segmentPairs[(i * 2) + 1] = (i % 2 == 0)

                // Crosses the first segment near its midpoint.
                ? new Segment(new Vertex(x, y + 10D), new Vertex(x + 10D, y))

                // Sits well clear of the first segment.
                : new Segment(new Vertex(x + 40D, y), new Vertex(x + 50D, y + 10D));
        }

        this.eventPairs = new SweepEvent[BatchSize * 2];
        SweepEventComparer comparer = new();
        for (int i = 0; i < BatchSize; i++)
        {
            this.eventPairs[i * 2] = LeftEventOf(this.segmentPairs[i * 2], PolygonType.Subject, i, comparer);
            this.eventPairs[(i * 2) + 1] = LeftEventOf(this.segmentPairs[(i * 2) + 1], PolygonType.Clipping, i, comparer);
        }
    }

    /// <summary>
    /// The orientation predicate. Called from both comparers and from <c>SweepEvent.Below</c>.
    /// </summary>
    /// <returns>The accumulated areas, returned so the loop cannot be optimised away.</returns>
    [Benchmark]
    public double SignedArea()
    {
        Vertex[] points = this.triples;
        double total = 0D;
        for (int i = 0; i < points.Length; i += 3)
        {
            total += PolygonUtilities.SignedArea(points[i], points[i + 1], points[i + 2]);
        }

        return total;
    }

    /// <summary>
    /// Segment intersection, reached once per <c>PossibleIntersection</c> and again from
    /// <c>SegmentComparer</c> whenever two segments straddle one another.
    /// </summary>
    /// <returns>The accumulated intersection counts.</returns>
    [Benchmark]
    public int FindIntersection()
    {
        Segment[] segments = this.segmentPairs;
        int total = 0;
        for (int i = 0; i < segments.Length; i += 2)
        {
            total += PolygonUtilities.FindIntersection(segments[i], segments[i + 1], out Vertex _, out Vertex _);
        }

        return total;
    }

    /// <summary>
    /// The event queue ordering, called on every heap sift.
    /// </summary>
    /// <returns>The accumulated comparison results.</returns>
    [Benchmark]
    public int SweepEventCompare()
    {
        SweepEvent[] events = this.eventPairs;
        SweepEventComparer comparer = new();
        int total = 0;
        for (int i = 0; i < events.Length; i += 2)
        {
            total += comparer.Compare(events[i], events[i + 1]);
        }

        return total;
    }

    /// <summary>
    /// The status line ordering. Measured on segment pairs that genuinely overlap in x, which is
    /// the only situation in which the sweep asks it anything.
    /// </summary>
    /// <returns>The accumulated comparison results.</returns>
    [Benchmark]
    public int SegmentCompare()
    {
        SweepEvent[] events = this.eventPairs;
        SegmentComparer comparer = new();
        int total = 0;
        for (int i = 0; i < events.Length; i += 2)
        {
            total += comparer.Compare(events[i], events[i + 1]);
        }

        return total;
    }

    /// <summary>
    /// Fills and drains the event queue, exercising both sift directions.
    /// </summary>
    /// <returns>The number of events drained.</returns>
    [Benchmark]
    public int PriorityQueueRoundTrip()
    {
        SweepEvent[] events = this.eventPairs;
        StablePriorityQueue<SweepEvent, SweepEventComparer> queue = new(new SweepEventComparer(), events.Length);

        for (int i = 0; i < events.Length; i++)
        {
            queue.Enqueue(events[i]);
        }

        int count = 0;
        while (queue.Count > 0)
        {
            _ = queue.Dequeue();
            count++;
        }

        return count;
    }

    /// <summary>
    /// Builds the left event of a segment the same way <c>PolygonClipper.ProcessSegment</c> does,
    /// so the benchmarked comparisons see the same shape of data the algorithm produces.
    /// </summary>
    private static SweepEvent LeftEventOf(Segment segment, PolygonType type, int contourId, SweepEventComparer comparer)
    {
        SweepEvent e1 = new(segment.Source, true, type);
        SweepEvent e2 = new(segment.Target, true, e1, type);
        e1.OtherEvent = e2;
        e1.ContourId = e2.ContourId = contourId;

        if (comparer.Compare(e1, e2) < 0)
        {
            e2.Left = false;
            return e1;
        }

        e1.Left = false;
        return e2;
    }
}
