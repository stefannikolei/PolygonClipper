// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace PolygonClipper;

/// <summary>
/// Triangulates polygons using the ear slicing algorithm, optionally refining the result toward
/// the constrained Delaunay triangulation.
/// </summary>
/// <remarks>
/// <para>
/// This is a port of the earcut library by Volodymyr Agafonkin
/// (<see href="https://github.com/mapbox/earcut"/>), which implements a fast ear clipping
/// triangulator with hole elimination, z-order curve hashing and handling for the degenerate
/// inputs found in real world geometry.
/// </para>
/// <para>
/// The triangulation is returned as triplets of vertex indices, which makes the result directly
/// consumable by indexed rendering APIs.
/// </para>
/// </remarks>
public static class PolygonTriangulator
{
    [ThreadStatic]
    private static EarcutEngine? engine;

    [ThreadStatic]
    private static DelaunayRefiner? refiner;

    /// <summary>
    /// Gets the triangulation engine of the current thread.
    /// </summary>
    private static EarcutEngine Engine => engine ??= new EarcutEngine();

    /// <summary>
    /// Gets the refinement engine of the current thread.
    /// </summary>
    private static DelaunayRefiner Refiner => refiner ??= new DelaunayRefiner();

    /// <summary>
    /// Triangulates the given polygon.
    /// </summary>
    /// <remarks>
    /// <para>
    /// Holes are taken from the polygon itself: every external contour, as reported by
    /// <see cref="Contour.IsExternal"/>, is triangulated together with the hole contours that
    /// follow it - the layout produced by <see cref="PolygonClipper"/>. A polygon may therefore
    /// contain any number of disjoint external contours.
    /// </para>
    /// <para>
    /// Polygons that carry no hole information, such as rings converted straight from GeoJSON, are
    /// treated as a set of disjoint external contours. To triangulate those with the first contour
    /// as the external contour and the remaining ones as its holes, pass
    /// <see cref="Flatten(Polygon)"/> to
    /// <see cref="Triangulate(ReadOnlySpan{double}, ReadOnlySpan{int}, int)"/> instead.
    /// </para>
    /// <para>
    /// The returned indices address the vertices of the polygon as a single sequence, with the
    /// contours concatenated in order. Use <see cref="Flatten(Polygon)"/> to obtain the matching
    /// coordinates.
    /// </para>
    /// </remarks>
    /// <param name="polygon">The polygon to triangulate.</param>
    /// <returns>The triangles as triplets of vertex indices.</returns>
    /// <exception cref="ArgumentNullException">The polygon is null.</exception>
    public static int[] Triangulate(Polygon polygon)
    {
        ArgumentNullException.ThrowIfNull(polygon);

        int contourCount = polygon.ContourCount;
        if (contourCount == 0)
        {
            return [];
        }

        // Triangulate each external contour together with the holes that follow it, mapping the
        // resulting indices back onto the polygon.
        double[] vertices = Flatten(polygon).Vertices;
        List<int> triangles = [];
        List<int> groupTriangles = [];
        List<int> holeIndices = [];
        EarcutEngine engine = Engine;

        int start = 0;
        int vertexOffset = 0;
        while (start < contourCount)
        {
            // Collect the external contour and the holes that follow it.
            int end = start + 1;
            while (end < contourCount && !polygon[end].IsExternal)
            {
                end++;
            }

            int vertexCount = 0;
            holeIndices.Clear();
            for (int i = start; i < end; i++)
            {
                if (i > start)
                {
                    holeIndices.Add(vertexCount);
                }

                vertexCount += polygon[i].VertexCount;
            }

            groupTriangles.Clear();
            engine.Triangulate(
                vertices.AsSpan(vertexOffset * 2, vertexCount * 2),
                CollectionsMarshal.AsSpan(holeIndices),
                2,
                groupTriangles);

            for (int i = 0; i < groupTriangles.Count; i++)
            {
                triangles.Add(groupTriangles[i] + vertexOffset);
            }

            start = end;
            vertexOffset += vertexCount;
        }

        return [.. triangles];
    }

    /// <summary>
    /// Triangulates a polygon given as a sequence of vertices.
    /// </summary>
    /// <param name="vertices">
    /// The vertices of the polygon; the external contour followed by any holes.
    /// </param>
    /// <param name="holeIndices">The indices, in vertices, where each hole ring starts.</param>
    /// <returns>The triangles as triplets of vertex indices.</returns>
    public static int[] Triangulate(ReadOnlySpan<Vertex> vertices, ReadOnlySpan<int> holeIndices)
        => Triangulate(MemoryMarshal.Cast<Vertex, double>(vertices), holeIndices, 2);

    /// <summary>
    /// Triangulates a polygon given as a flat array of vertex coordinates.
    /// </summary>
    /// <param name="data">
    /// The flat array of vertex coordinates; the external contour followed by any holes.
    /// </param>
    /// <param name="holeIndices">
    /// The indices, in vertices rather than coordinates, where each hole ring starts.
    /// </param>
    /// <param name="dimensions">The number of coordinates per vertex in <paramref name="data"/>.</param>
    /// <returns>The triangles as triplets of vertex indices.</returns>
    /// <example>
    /// <code>
    /// int[] triangles = PolygonTriangulator.Triangulate([10, 0, 0, 50, 60, 60, 70, 10], []);
    /// </code>
    /// </example>
    public static int[] Triangulate(ReadOnlySpan<double> data, ReadOnlySpan<int> holeIndices, int dimensions = 2)
    {
        List<int> triangles = [];
        Engine.Triangulate(data, holeIndices, dimensions, triangles);
        return [.. triangles];
    }

    /// <summary>
    /// Refines a triangulation toward the constrained Delaunay triangulation by legalizing every
    /// interior edge in place with Lawson flips - maximizing the minimum angle and removing most
    /// slivers.
    /// </summary>
    /// <remarks>
    /// An optional post-pass for <see cref="Triangulate(Polygon)"/> output, or any manifold
    /// triangle index array indexing into the vertices.
    /// </remarks>
    /// <param name="triangles">The triangle indices, mutated in place.</param>
    /// <param name="vertices">The vertices the triangles index into.</param>
    public static void Refine(Span<int> triangles, ReadOnlySpan<Vertex> vertices)
        => Refine(triangles, MemoryMarshal.Cast<Vertex, double>(vertices), 2);

    /// <summary>
    /// Refines a triangulation toward the constrained Delaunay triangulation by legalizing every
    /// interior edge in place with Lawson flips - maximizing the minimum angle and removing most
    /// slivers.
    /// </summary>
    /// <param name="triangles">The triangle indices, mutated in place.</param>
    /// <param name="coords">The flat vertex coordinates the triangles index into.</param>
    /// <param name="dimensions">The number of coordinates per vertex in <paramref name="coords"/>.</param>
    public static void Refine(Span<int> triangles, ReadOnlySpan<double> coords, int dimensions = 2)
        => Refiner.Refine(triangles, coords, dimensions);

    /// <summary>
    /// Returns the relative difference between the polygon area and the area of its triangulation.
    /// A value near 0 means a correct triangulation.
    /// </summary>
    /// <remarks>
    /// The contours of the polygon are interpreted as by <see cref="Triangulate(Polygon)"/>:
    /// external contours add to the area, the hole contours following them subtract from it.
    /// </remarks>
    /// <param name="polygon">The triangulated polygon.</param>
    /// <param name="triangles">The triangulation of the polygon.</param>
    /// <returns>The relative area difference.</returns>
    /// <exception cref="ArgumentNullException">The polygon is null.</exception>
    public static double Deviation(Polygon polygon, ReadOnlySpan<int> triangles)
    {
        ArgumentNullException.ThrowIfNull(polygon);

        FlatPolygon flat = Flatten(polygon);

        // External contours add to the area, holes subtract from it.
        double polygonArea = 0;
        int offset = 0;
        for (int i = 0; i < polygon.ContourCount; i++)
        {
            Contour contour = polygon[i];
            int count = contour.VertexCount;
            double area = Math.Abs(SignedArea(flat.Vertices, offset * 2, (offset + count) * 2, 2));
            polygonArea += contour.IsExternal ? area : -area;
            offset += count;
        }

        return Deviation(flat.Vertices, 2, triangles, polygonArea);
    }

    /// <summary>
    /// Returns the relative difference between the polygon area and the area of its triangulation.
    /// A value near 0 means a correct triangulation.
    /// </summary>
    /// <param name="data">The flat array of vertex coordinates.</param>
    /// <param name="holeIndices">
    /// The indices, in vertices rather than coordinates, where each hole ring starts.
    /// </param>
    /// <param name="dimensions">The number of coordinates per vertex in <paramref name="data"/>.</param>
    /// <param name="triangles">The triangulation of the polygon.</param>
    /// <returns>The relative area difference.</returns>
    public static double Deviation(
        ReadOnlySpan<double> data,
        ReadOnlySpan<int> holeIndices,
        int dimensions,
        ReadOnlySpan<int> triangles)
    {
        bool hasHoles = holeIndices.Length > 0;
        int outerLen = hasHoles ? holeIndices[0] * dimensions : data.Length;

        double polygonArea = Math.Abs(SignedArea(data, 0, outerLen, dimensions));
        if (hasHoles)
        {
            for (int i = 0, len = holeIndices.Length; i < len; i++)
            {
                int start = holeIndices[i] * dimensions;
                int end = i < len - 1 ? holeIndices[i + 1] * dimensions : data.Length;
                polygonArea -= Math.Abs(SignedArea(data, start, end, dimensions));
            }
        }

        return Deviation(data, dimensions, triangles, polygonArea);
    }

    /// <summary>
    /// Turns a polygon into the flat form the triangulator accepts.
    /// </summary>
    /// <param name="polygon">The polygon to flatten.</param>
    /// <returns>The <see cref="FlatPolygon"/>.</returns>
    /// <exception cref="ArgumentNullException">The polygon is null.</exception>
    public static FlatPolygon Flatten(Polygon polygon)
    {
        ArgumentNullException.ThrowIfNull(polygon);

        int contourCount = polygon.ContourCount;
        double[] vertices = new double[polygon.GetVertexCount() * 2];
        int[] holeIndices = new int[Math.Max(contourCount - 1, 0)];

        int index = 0;
        int vertexIndex = 0;
        for (int i = 0; i < contourCount; i++)
        {
            if (i > 0)
            {
                holeIndices[i - 1] = vertexIndex;
            }

            Contour contour = polygon[i];
            for (int j = 0; j < contour.VertexCount; j++)
            {
                Vertex vertex = contour.GetVertex(j);
                vertices[index++] = vertex.X;
                vertices[index++] = vertex.Y;
                vertexIndex++;
            }
        }

        return new FlatPolygon(vertices, holeIndices, 2);
    }

    /// <summary>
    /// Calculates the signed area of the ring within the given range.
    /// </summary>
    /// <param name="data">The flat array of vertex coordinates.</param>
    /// <param name="start">The inclusive start offset into <paramref name="data"/>.</param>
    /// <param name="end">The exclusive end offset into <paramref name="data"/>.</param>
    /// <param name="dim">The number of coordinates per vertex in <paramref name="data"/>.</param>
    /// <returns>The signed area.</returns>
    internal static double SignedArea(ReadOnlySpan<double> data, int start, int end, int dim)
    {
        double sum = 0;
        for (int i = start, j = end - dim; i < end; i += dim)
        {
            sum += (data[j] - data[i]) * (data[i + 1] + data[j + 1]);
            j = i;
        }

        return sum;
    }

    /// <summary>
    /// Compares the area of the triangulation against the area of the polygon it was created from.
    /// </summary>
    /// <param name="data">The flat array of vertex coordinates.</param>
    /// <param name="dim">The number of coordinates per vertex in <paramref name="data"/>.</param>
    /// <param name="triangles">The triangulation of the polygon.</param>
    /// <param name="polygonArea">The area of the polygon.</param>
    /// <returns>The relative area difference.</returns>
    private static double Deviation(ReadOnlySpan<double> data, int dim, ReadOnlySpan<int> triangles, double polygonArea)
    {
        double trianglesArea = 0;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            int a = triangles[i] * dim;
            int b = triangles[i + 1] * dim;
            int c = triangles[i + 2] * dim;
            trianglesArea += Math.Abs(
                ((data[a] - data[c]) * (data[b + 1] - data[a + 1])) -
                ((data[a] - data[b]) * (data[c + 1] - data[a + 1])));
        }

        return polygonArea == 0 && trianglesArea == 0
            ? 0
            : Math.Abs((trianglesArea - polygonArea) / polygonArea);
    }
}
