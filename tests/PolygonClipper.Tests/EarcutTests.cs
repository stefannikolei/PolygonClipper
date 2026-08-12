// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections.Generic;
using System.Linq;
using PolygonClipper.Tests.TestCases;
using Xunit;
using Xunit.Abstractions;

namespace PolygonClipper.Tests;

/// <summary>
/// Tests the triangulation, ported from the test suite of
/// <see href="https://github.com/mapbox/earcut"/>.
/// </summary>
public class EarcutTests
{
    private static readonly int[] Rotations = [0, 90, 180, 270];

    private readonly ITestOutputHelper testOutputHelper;

    public EarcutTests(ITestOutputHelper testOutputHelper) => this.testOutputHelper = testOutputHelper;

    public static IEnumerable<object[]> GetFixtureCases()
        => from name in EarcutTestData.GetFixtureNames()
           from rotation in Rotations
           select new object[] { name, rotation };

    [Fact]
    public void Indices2D()
    {
        int[] indices = PolygonTriangulator.Triangulate([10, 0, 0, 50, 60, 60, 70, 10], []);

        Assert.Equal([1, 0, 3, 1, 3, 2], indices);
    }

    [Fact]
    public void Indices3D()
    {
        int[] indices = PolygonTriangulator.Triangulate([10, 0, 0, 0, 50, 0, 60, 60, 0, 70, 10, 0], [], 3);

        Assert.Equal([1, 0, 3, 1, 3, 2], indices);
    }

    [Fact]
    public void Empty()
    {
        int[] indices = PolygonTriangulator.Triangulate([], []);

        Assert.Empty(indices);
    }

    [Theory]
    [MemberData(nameof(GetFixtureCases))]
    public void Fixture(string name, int rotation)
    {
        // Arrange
        double[][][] rings = EarcutTestData.GetRings(name);
        Polygon polygon = EarcutTestHelpers.CreatePolygon(rings, rotation);
        FlatPolygon data = PolygonTriangulator.Flatten(polygon);

        // Act
        int[] indices = PolygonTriangulator.Triangulate(data.Vertices, data.HoleIndices, data.Dimensions);

        // Assert
        double err = PolygonTriangulator.Deviation(data.Vertices, data.HoleIndices, data.Dimensions, indices);
        int expectedTriangles = EarcutTestData.Expectations.Triangles[name];
        double expectedDeviation = EarcutTestData.Expectations.GetExpectedDeviation(name, rotation);

        int numTriangles = indices.Length / 3;
        if (rotation == 0)
        {
            Assert.True(
                numTriangles == expectedTriangles,
                $"{numTriangles} triangles when expected {expectedTriangles}");
        }

        if (expectedTriangles > 0)
        {
            Assert.True(err <= expectedDeviation, $"deviation {err} <= {expectedDeviation}");
        }

        // Surface fixtures whose deviation is well below the recorded threshold (at least 3x), so
        // improvements after a correctness fix are visible and the threshold can be tightened.
        if (expectedDeviation > 0 && err * 3 < expectedDeviation)
        {
            this.testOutputHelper.WriteLine(
                $"{name}: deviation {err} < recorded {expectedDeviation} (improved)");
        }
    }

    [Fact]
    public void InfiniteLoop()
        => PolygonTriangulator.Triangulate([1, 2, 2, 2, 1, 2, 1, 1, 1, 2, 4, 1, 5, 1, 3, 2, 4, 2, 4, 1], [5], 2);

    /// <summary>
    /// Regression for the hole-bridge block index (issue #183): a collinear-rich outer ring
    /// (integer grid, like vector tile data) plus multiple holes used to drop a hole when
    /// filtering healed a collinear run across a block boundary, leaving the surviving edge
    /// outside its block's stale bounding box so the leftward-ray scan false-skipped it. Assert
    /// full coverage.
    /// </summary>
    /// <param name="rotation">The rotation in degrees.</param>
    [Theory]
    [InlineData(0)]
    [InlineData(90)]
    [InlineData(180)]
    [InlineData(270)]
    public void BlockIndexCollinear(int rotation)
    {
        const int N = 30;
        List<double[]> outer = [];
        for (int x = 0; x <= N; x++)
        {
            outer.Add([x, 0]);
        }

        for (int y = 1; y <= N; y++)
        {
            outer.Add([N, y]);
        }

        for (int x = N - 1; x >= 0; x--)
        {
            outer.Add([x, N]);
        }

        for (int y = N - 1; y >= 1; y--)
        {
            outer.Add([0, y]);
        }

        static double[][] Rect(double x0, double y0, double w, double h)
            => [[x0, y0], [x0, y0 + h], [x0 + w, y0 + h], [x0 + w, y0]];

        double[][][] rings = [[.. outer], Rect(5, 5, 2, 4), Rect(2, 23, 1, 1)];

        Polygon polygon = EarcutTestHelpers.CreatePolygon(rings, rotation);
        FlatPolygon data = PolygonTriangulator.Flatten(polygon);
        int[] indices = PolygonTriangulator.Triangulate(data.Vertices, data.HoleIndices, data.Dimensions);
        double err = PolygonTriangulator.Deviation(data.Vertices, data.HoleIndices, data.Dimensions, indices);

        Assert.True(err < 1e-9, $"rotation {rotation}: deviation {err} (hole dropped?)");
    }
}
