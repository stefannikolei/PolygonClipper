// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections.Generic;
using System.Linq;
using GeoJSON.Text.Feature;
using PolygonClipper.Tests.TestCases;
using Xunit;

namespace PolygonClipper.Tests;

/// <summary>
/// Tests the triangulation API against the polygon types of this library.
/// </summary>
public class PolygonTriangulatorTests
{
    /// <summary>
    /// Clipping results whose hole information is known to be wrong, so the area they cover cannot
    /// be derived from their contours. In <c>overlapping_segments1</c> the union of four disjoint
    /// triangles marks the second contour as a hole of the first, although the two do not even
    /// share a bounding box. This is a limitation of the clipper, not of the triangulation.
    /// </summary>
    private static readonly string[] IncorrectHoleInformation = ["overlapping_segments1.geojson"];

    public static IEnumerable<object[]> GetTestCases()
        => TestData.Generic.GetFileNames()
            .Where(x => !IncorrectHoleInformation.Contains(x))
            .Select(x => new object[] { x });

    [Fact]
    public void TriangulateEmptyPolygonReturnsNoTriangles()
        => Assert.Empty(PolygonTriangulator.Triangulate(new Polygon()));

    [Fact]
    public void TriangulateVertices()
    {
        Vertex[] vertices = [new(10, 0), new(0, 50), new(60, 60), new(70, 10)];

        int[] triangles = PolygonTriangulator.Triangulate(vertices, []);

        Assert.Equal([1, 0, 3, 1, 3, 2], triangles);
    }

    [Fact]
    public void TriangulateContourWithHole()
    {
        // A square with a square hole. Holes are the contours marked as such.
        Polygon polygon = new();
        polygon.Push(CreateContour([(0, 0), (10, 0), (10, 10), (0, 10)]));

        Contour hole = CreateContour([(3, 3), (3, 7), (7, 7), (7, 3)]);
        hole.HoleOf = 0;
        polygon.Push(hole);
        polygon[0].AddHoleIndex(1);

        int[] triangles = PolygonTriangulator.Triangulate(polygon);

        Assert.Equal(8 * 3, triangles.Length);
        Assert.Equal(0, PolygonTriangulator.Deviation(polygon, triangles));
        AssertIndicesInRange(polygon, triangles);
    }

    [Fact]
    public void TriangulateMultipleExternalContours()
    {
        // Two disjoint squares, the first one with a hole, described the way the clipper describes
        // its results.
        Polygon polygon = new();
        polygon.Push(CreateContour([(0, 0), (10, 0), (10, 10), (0, 10)]));

        Contour hole = CreateContour([(3, 3), (3, 7), (7, 7), (7, 3)]);
        hole.HoleOf = 0;
        polygon.Push(hole);
        polygon[0].AddHoleIndex(1);

        polygon.Push(CreateContour([(20, 0), (30, 0), (30, 10), (20, 10)]));

        int[] triangles = PolygonTriangulator.Triangulate(polygon);

        // 8 triangles for the square with the hole, 2 for the plain square.
        Assert.Equal(10 * 3, triangles.Length);
        Assert.Equal(0, PolygonTriangulator.Deviation(polygon, triangles));
        AssertIndicesInRange(polygon, triangles);
    }

    [Fact]
    public void TriangulateRingsWithoutHoleInformation()
    {
        // Rings without hole information, as converted straight from GeoJSON, are triangulated
        // with the earcut convention by flattening them first: the first ring is the external
        // contour, the remaining rings are its holes.
        Polygon polygon = new();
        polygon.Push(CreateContour([(0, 0), (10, 0), (10, 10), (0, 10)]));
        polygon.Push(CreateContour([(3, 3), (3, 7), (7, 7), (7, 3)]));

        FlatPolygon flat = PolygonTriangulator.Flatten(polygon);
        int[] triangles = PolygonTriangulator.Triangulate(flat.Vertices, flat.HoleIndices, flat.Dimensions);

        Assert.Equal(8 * 3, triangles.Length);
        Assert.Equal(
            0,
            PolygonTriangulator.Deviation(flat.Vertices, flat.HoleIndices, flat.Dimensions, triangles));

        // Interpreted as separate external contours instead, the hole is filled in as well.
        Assert.Equal(4 * 3, PolygonTriangulator.Triangulate(polygon).Length);
    }

    [Theory]
    [MemberData(nameof(GetTestCases))]
    public void TriangulateClippingResult(string testCaseFile)
    {
        // Arrange
        FeatureCollection data = TestData.Generic.GetFeatureCollection(testCaseFile);
        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(data);

        // Act
        Polygon union = PolygonClipper.Union(subject, clipping);
        int[] triangles = PolygonTriangulator.Triangulate(union);

        // Assert
        if (union.GetVertexCount() == 0)
        {
            Assert.Empty(triangles);
            return;
        }

        Assert.Equal(0, triangles.Length % 3);
        AssertIndicesInRange(union, triangles);

        // The triangles must cover the same area as the polygon they were built from.
        Assert.True(
            PolygonTriangulator.Deviation(union, triangles) < 1e-9,
            $"deviation {PolygonTriangulator.Deviation(union, triangles)}");
    }

    private static Contour CreateContour(ReadOnlySpan<(double X, double Y)> vertices)
    {
        Contour contour = new();
        foreach ((double x, double y) in vertices)
        {
            contour.AddVertex(new Vertex(x, y));
        }

        return contour;
    }

    private static void AssertIndicesInRange(Polygon polygon, int[] triangles)
    {
        int vertexCount = polygon.GetVertexCount();
        foreach (int index in triangles)
        {
            Assert.InRange(index, 0, vertexCount - 1);
        }
    }
}
