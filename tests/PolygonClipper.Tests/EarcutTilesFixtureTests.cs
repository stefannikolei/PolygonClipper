// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections.Generic;
using PolygonClipper.Tests.TestCases;
using Xunit;

namespace PolygonClipper.Tests;

/// <summary>
/// Triangulates every polygon of a real world vector tile corpus, ported from the test suite of
/// <see href="https://github.com/mapbox/earcut"/>.
/// </summary>
public class EarcutTilesFixtureTests
{
    [Fact]
    public void MvtFixtureHasZeroDeviationAndRefinedQuality()
    {
        List<MvtPolygon> polys = MvtTilesFixture.Read(EarcutTestData.GetTilesFixturePath());
        int nonzero = 0;
        int firstIndex = -1;
        double firstDev = 0;
        int worstIndex = -1;
        double worstDev = 0;
        double sumDev = 0;
        int refinedNonzero = 0;
        int refinedFirstIndex = -1;
        double refinedFirstDev = 0;
        int refinedWorstIndex = -1;
        double refinedWorstDev = 0;
        double refinedSumDev = 0;
        int lengthChanged = 0;
        double basePerimeter = 0;
        double refinedPerimeter = 0;

        for (int i = 0; i < polys.Count; i++)
        {
            FlatPolygon data = polys[i].Data;
            int[] triangles = PolygonTriangulator.Triangulate(data.Vertices, data.HoleIndices, data.Dimensions);
            int length = triangles.Length;
            basePerimeter += EarcutTestHelpers.TrianglePerimeter(triangles, data.Vertices, data.Dimensions);
            double dev = PolygonTriangulator.Deviation(data.Vertices, data.HoleIndices, data.Dimensions, triangles);
            if (dev != 0)
            {
                if (firstIndex < 0)
                {
                    firstIndex = i;
                    firstDev = dev;
                }

                nonzero++;
                sumDev += dev;
                if (dev > worstDev)
                {
                    worstIndex = i;
                    worstDev = dev;
                }
            }

            PolygonTriangulator.Refine(triangles, data.Vertices, data.Dimensions);
            refinedPerimeter += EarcutTestHelpers.TrianglePerimeter(triangles, data.Vertices, data.Dimensions);
            if (triangles.Length != length)
            {
                lengthChanged++;
            }

            double refinedDev = PolygonTriangulator.Deviation(
                data.Vertices,
                data.HoleIndices,
                data.Dimensions,
                triangles);

            if (refinedDev != 0)
            {
                if (refinedFirstIndex < 0)
                {
                    refinedFirstIndex = i;
                    refinedFirstDev = refinedDev;
                }

                refinedNonzero++;
                refinedSumDev += refinedDev;
                if (refinedDev > refinedWorstDev)
                {
                    refinedWorstIndex = i;
                    refinedWorstDev = refinedDev;
                }
            }
        }

        Assert.Equal(119680, polys.Count);
        Assert.True(
            nonzero == 0,
            $"{nonzero} polygons with nonzero deviation; first {firstIndex}: {firstDev}, " +
            $"worst {worstIndex}: {worstDev}, sum {sumDev}");

        Assert.True(lengthChanged == 0, $"{lengthChanged} refined triangulations changed triangle count");
        Assert.True(
            refinedNonzero == 0,
            $"{refinedNonzero} refined polygons with nonzero deviation; first {refinedFirstIndex}: " +
            $"{refinedFirstDev}, worst {refinedWorstIndex}: {refinedWorstDev}, sum {refinedSumDev}");

        Assert.True(
            refinedPerimeter < basePerimeter * 0.72,
            $"refined perimeter ratio {refinedPerimeter / basePerimeter} < 0.72");
    }
}
