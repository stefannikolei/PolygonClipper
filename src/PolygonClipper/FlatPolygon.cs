// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace PolygonClipper;

/// <summary>
/// Represents a polygon in the flat form consumed by
/// <see cref="PolygonTriangulator.Triangulate(System.ReadOnlySpan{double}, System.ReadOnlySpan{int}, int)"/>.
/// </summary>
public readonly struct FlatPolygon
{
    /// <summary>
    /// Initializes a new instance of the <see cref="FlatPolygon"/> struct.
    /// </summary>
    /// <param name="vertices">The flat array of vertex coordinates.</param>
    /// <param name="holeIndices">The indices, in vertices, where each hole ring starts.</param>
    /// <param name="dimensions">The number of coordinates per vertex.</param>
    public FlatPolygon(double[] vertices, int[] holeIndices, int dimensions)
    {
        this.Vertices = vertices;
        this.HoleIndices = holeIndices;
        this.Dimensions = dimensions;
    }

    /// <summary>
    /// Gets the flat array of vertex coordinates.
    /// </summary>
#pragma warning disable CA1819 // Properties should not return arrays
    public double[] Vertices { get; }

    /// <summary>
    /// Gets the indices, in vertices rather than coordinates, where each hole ring starts.
    /// </summary>
    public int[] HoleIndices { get; }
#pragma warning restore CA1819

    /// <summary>
    /// Gets the number of coordinates per vertex in <see cref="Vertices"/>.
    /// </summary>
    public int Dimensions { get; }
}
