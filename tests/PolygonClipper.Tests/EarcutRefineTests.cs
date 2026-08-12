// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using PolygonClipper.Tests.TestCases;
using Xunit;

namespace PolygonClipper.Tests;

/// <summary>
/// Tests the Delaunay refinement of a triangulation, ported from the test suite of
/// <see href="https://github.com/mapbox/earcut"/>.
/// </summary>
public class EarcutRefineTests
{
    [Fact]
    public void RefineImprovesABadQuadDiagonal()
    {
        double[] vertices = [0, 0, 3, 0, 10, 1, 0, 2];
        int[] triangles = [2, 3, 0, 2, 0, 1];
        double beforePerimeter = EarcutTestHelpers.TrianglePerimeter(triangles, vertices);

        PolygonTriangulator.Refine(triangles, vertices);

        double afterPerimeter = EarcutTestHelpers.TrianglePerimeter(triangles, vertices);

        Assert.Equal([2, 3, 1, 3, 0, 1], triangles);
        Assert.True(afterPerimeter < beforePerimeter * 0.7);
        Assert.Equal(0, PolygonTriangulator.Deviation(vertices, [], 2, triangles));
    }

    [Fact]
    public void RefineLeavesAGoodQuadDiagonalAlone()
    {
        double[] vertices = [0, 0, 5, 0, 4, 1, 0, 4];
        int[] triangles = [2, 3, 0, 2, 0, 1];

        PolygonTriangulator.Refine(triangles, vertices);

        Assert.Equal([2, 3, 0, 2, 0, 1], triangles);
        Assert.Equal(0, PolygonTriangulator.Deviation(vertices, [], 2, triangles));
    }

    [Fact]
    public void RefinePreservesAConcavePolygon()
    {
        double[] vertices = [0, 0, 4, 0, 4, 1, 1, 1, 1, 4, 0, 4];
        int[] triangles = PolygonTriangulator.Triangulate(vertices, []);
        int length = triangles.Length;
        double beforePerimeter = EarcutTestHelpers.TrianglePerimeter(triangles, vertices);

        PolygonTriangulator.Refine(triangles, vertices);

        double afterPerimeter = EarcutTestHelpers.TrianglePerimeter(triangles, vertices);

        Assert.Equal(length, triangles.Length);
        Assert.True(afterPerimeter < beforePerimeter * 0.9);
        Assert.Equal(0, PolygonTriangulator.Deviation(vertices, [], 2, triangles));
    }

    /// <summary>
    /// Four near-cocircular points make the non-robust incircle test give inconsistent signs for
    /// an edge and its flip; without the tie margin the Lawson cascade flips one edge back and
    /// forth forever. Should return with a valid mesh of unchanged size and zero deviation.
    /// </summary>
    [Fact(Timeout = 5000)]
    public void RefineTerminatesOnNearCocircularPoints()
    {
        double[] vertices =
        [
            127.65906365022843, 9.336137742499535, 124.21725103117963, 30.888097161477972,
            91.35514946628345, 89.65621376119454, 40.10446780041529, 121.5550560957686,
            -110.83205604043928, 64.03323632184248, -127.20394987965459, -14.253249980770189,
            61.074962259031416, -112.48932831632469, 127.37846573978545, -12.598669206638515,
            127.77010311801033, -7.668164657400608
        ];

        int[] triangles = PolygonTriangulator.Triangulate(vertices, []);
        int length = triangles.Length;

        PolygonTriangulator.Refine(triangles, vertices);

        Assert.Equal(length, triangles.Length);
        Assert.True(PolygonTriangulator.Deviation(vertices, [], 2, triangles) < 1e-15);
    }

    [Fact]
    public void RefineLegalizesAllConvexInteriorEdgesInEarcutFixture()
    {
        double[][][] rings = EarcutTestData.GetRings("earcut");
        Polygon polygon = EarcutTestHelpers.CreatePolygon(rings, 0);
        FlatPolygon data = PolygonTriangulator.Flatten(polygon);
        int[] triangles = PolygonTriangulator.Triangulate(data.Vertices, data.HoleIndices, data.Dimensions);

        PolygonTriangulator.Refine(triangles, data.Vertices, data.Dimensions);

        Assert.Equal(0, EarcutTestHelpers.CountIllegalEdges(triangles, data.Vertices, data.Dimensions));
        Assert.Equal(
            0,
            PolygonTriangulator.Deviation(data.Vertices, data.HoleIndices, data.Dimensions, triangles));
    }
}
