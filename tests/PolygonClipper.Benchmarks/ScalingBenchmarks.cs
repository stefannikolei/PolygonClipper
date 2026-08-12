// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;

namespace PolygonClipper.Benchmarks;

/// <summary>
/// Measures how a union scales with the input size across three shape families that load the
/// algorithm differently. See <see cref="SyntheticPolygons"/> for what each family stresses.
/// </summary>
[MemoryDiagnoser]
public class ScalingBenchmarks
{
    private Polygon combSubject = null!;
    private Polygon combClipping = null!;
    private Polygon circleSubject = null!;
    private Polygon circleClipping = null!;
    private Polygon gridSubject = null!;
    private Polygon gridClipping = null!;
    private Polygon barsSubject = null!;
    private Polygon barsClipping = null!;

    /// <summary>
    /// Gets or sets the approximate combined vertex count of the two input polygons.
    /// </summary>
    [Params(1_000, 10_000, 50_000)]
    public int VertexCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        (this.combSubject, this.combClipping) = SyntheticPolygons.InterlockingCombs(this.VertexCount);
        (this.circleSubject, this.circleClipping) = SyntheticPolygons.OverlappingCircles(this.VertexCount);
        (this.gridSubject, this.gridClipping) = SyntheticPolygons.SharedEdgeGrid(this.VertexCount);
        (this.barsSubject, this.barsClipping) = SyntheticPolygons.StackedBars(this.VertexCount);
    }

    /// <summary>
    /// Intersection count grows with the vertex count.
    /// </summary>
    /// <returns>The union of the two combs.</returns>
    [Benchmark]
    public Polygon InterlockingCombs()
        => global::PolygonClipper.PolygonClipper.Union(this.combSubject, this.combClipping);

    /// <summary>
    /// Vertex count grows but the intersection count stays at two.
    /// </summary>
    /// <returns>The union of the two circles.</returns>
    [Benchmark]
    public Polygon OverlappingCircles()
        => global::PolygonClipper.PolygonClipper.Union(this.circleSubject, this.circleClipping);

    /// <summary>
    /// Coincident edges throughout, which is the worst case for the comparers and for the
    /// result event ordering in <c>ConnectEdges</c>.
    /// </summary>
    /// <returns>The union of the two grids.</returns>
    [Benchmark]
    public Polygon SharedEdgeGrid()
        => global::PolygonClipper.PolygonClipper.Union(this.gridSubject, this.gridClipping);

    /// <summary>
    /// The only family here whose status line grows with the input: every bar edge spans every
    /// sweep position, so the status line holds roughly 2n entries at once instead of a handful.
    /// </summary>
    /// <returns>The union of the bar stack and its clipping rectangle.</returns>
    [Benchmark]
    public Polygon StackedBars()
        => global::PolygonClipper.PolygonClipper.Union(this.barsSubject, this.barsClipping);
}

/// <summary>
/// Compares the four boolean operations against each other at a single fixed size, so that a
/// change which only helps one of them cannot hide behind a union-only measurement.
/// </summary>
[MemoryDiagnoser]
public class OperationBenchmarks
{
    private Polygon subject = null!;
    private Polygon clipping = null!;

    [GlobalSetup]
    public void Setup()
        => (this.subject, this.clipping) = SyntheticPolygons.InterlockingCombs(10_000);

    /// <summary>
    /// Gets or sets the operation under test.
    /// </summary>
    [Params(
        BooleanOperation.Intersection,
        BooleanOperation.Union,
        BooleanOperation.Difference,
        BooleanOperation.Xor)]
    public BooleanOperation Operation { get; set; }

    /// <summary>
    /// Runs the configured boolean operation end to end.
    /// </summary>
    /// <returns>The resulting polygon.</returns>
    [Benchmark]
    public Polygon Run()
        => new global::PolygonClipper.PolygonClipper(this.subject, this.clipping, this.Operation).Run();
}
