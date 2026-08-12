// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections.Generic;

namespace PolygonClipper.Tests;

/// <summary>
/// Helpers shared by the triangulation tests, ported from
/// <see href="https://github.com/mapbox/earcut"/>.
/// </summary>
internal static class EarcutTestHelpers
{
    /// <summary>
    /// Rotates the given rings by the given angle, as the reference test suite does, and converts
    /// them into a <see cref="Polygon"/>.
    /// </summary>
    /// <param name="rings">The rings; the first is the external contour, the rest are holes.</param>
    /// <param name="rotation">The rotation in degrees.</param>
    /// <returns>The polygon.</returns>
    public static Polygon CreatePolygon(double[][][] rings, int rotation)
    {
        double theta = rotation * Math.PI / 180;
        double xx = Math.Round(Math.Cos(theta), MidpointRounding.AwayFromZero);
        double xy = Math.Round(-Math.Sin(theta), MidpointRounding.AwayFromZero);
        double yx = Math.Round(Math.Sin(theta), MidpointRounding.AwayFromZero);
        double yy = Math.Round(Math.Cos(theta), MidpointRounding.AwayFromZero);

        Polygon polygon = new();
        foreach (double[][] ring in rings)
        {
            Contour contour = new();
            foreach (double[] coord in ring)
            {
                double x = coord[0];
                double y = coord[1];
                contour.AddVertex(rotation == 0
                    ? new Vertex(x, y)
                    : new Vertex((xx * x) + (xy * y), (yx * x) + (yy * y)));
            }

            polygon.Push(contour);
        }

        return polygon;
    }

    /// <summary>
    /// Calculates the summed perimeter of all triangles.
    /// </summary>
    /// <param name="triangles">The triangle indices.</param>
    /// <param name="vertices">The flat vertex coordinates.</param>
    /// <param name="dim">The number of coordinates per vertex.</param>
    /// <returns>The perimeter.</returns>
    public static double TrianglePerimeter(ReadOnlySpan<int> triangles, ReadOnlySpan<double> vertices, int dim = 2)
    {
        double perimeter = 0;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            double ax = vertices[triangles[i] * dim];
            double ay = vertices[(triangles[i] * dim) + 1];
            double bx = vertices[triangles[i + 1] * dim];
            double by = vertices[(triangles[i + 1] * dim) + 1];
            double cx = vertices[triangles[i + 2] * dim];
            double cy = vertices[(triangles[i + 2] * dim) + 1];

            perimeter += double.Hypot(ax - bx, ay - by) +
                double.Hypot(bx - cx, by - cy) +
                double.Hypot(cx - ax, cy - ay);
        }

        return perimeter;
    }

    /// <summary>
    /// Counts the interior edges of a triangulation that violate the Delaunay condition even
    /// though the quad they belong to is convex.
    /// </summary>
    /// <param name="triangles">The triangle indices.</param>
    /// <param name="vertices">The flat vertex coordinates.</param>
    /// <param name="dim">The number of coordinates per vertex.</param>
    /// <returns>The number of illegal edges.</returns>
    public static int CountIllegalEdges(ReadOnlySpan<int> triangles, ReadOnlySpan<double> vertices, int dim = 2)
    {
        int[] halfEdges = new int[triangles.Length];
        Array.Fill(halfEdges, -1);
        Dictionary<(int A, int B), int> edges = [];

        for (int e = 0; e < triangles.Length; e++)
        {
            int a = triangles[e];
            int b = triangles[NextHalfEdge(e)];
            (int A, int B) key = a < b ? (a, b) : (b, a);
            if (edges.TryGetValue(key, out int twin))
            {
                halfEdges[e] = twin;
                halfEdges[twin] = e;
                edges.Remove(key);
            }
            else
            {
                edges[key] = e;
            }
        }

        int illegal = 0;
        for (int a = 0; a < triangles.Length; a++)
        {
            int b = halfEdges[a];
            if (b == -1 || a > b)
            {
                continue;
            }

            int a0 = a - (a % 3);
            int b0 = b - (b % 3);
            int ar = a0 + ((a + 2) % 3);
            int al = a0 + ((a + 1) % 3);
            int bl = b0 + ((b + 2) % 3);
            int p0 = triangles[ar], pr = triangles[a], pl = triangles[al], p1 = triangles[bl];

            double x0 = vertices[p0 * dim], y0 = vertices[(p0 * dim) + 1];
            double xr = vertices[pr * dim], yr = vertices[(pr * dim) + 1];
            double xl = vertices[pl * dim], yl = vertices[(pl * dim) + 1];
            double x1 = vertices[p1 * dim], y1 = vertices[(p1 * dim) + 1];
            bool convex = Orient(x0, y0, xr, yr, x1, y1) > 0 && Orient(x0, y0, x1, y1, xl, yl) > 0;

            if (convex && !InCircle(x0, y0, xr, yr, xl, yl, x1, y1))
            {
                illegal++;
            }
        }

        return illegal;
    }

    private static int NextHalfEdge(int e) => e - (e % 3) + ((e + 1) % 3);

    private static double Orient(double ax, double ay, double bx, double by, double cx, double cy)
        => ((bx - ax) * (cy - ay)) - ((by - ay) * (cx - ax));

    private static bool InCircle(
        double ax,
        double ay,
        double bx,
        double by,
        double cx,
        double cy,
        double px,
        double py)
    {
        double dx = ax - px, dy = ay - py, ex = bx - px, ey = by - py, fx = cx - px, fy = cy - py;
        double ap = (dx * dx) + (dy * dy), bp = (ex * ex) + (ey * ey), cp = (fx * fx) + (fy * fy);
        return (dx * ((ey * cp) - (bp * fy))) - (dy * ((ex * cp) - (bp * fx))) + (ap * ((ex * fy) - (ey * fx))) <= 0;
    }
}
