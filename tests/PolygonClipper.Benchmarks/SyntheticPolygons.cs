// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace PolygonClipper.Benchmarks;

/// <summary>
/// <para>
/// Generates polygon pairs of a caller controlled size.
/// </para>
/// <para>
/// The bundled GeoJSON fixtures top out at a few hundred vertices, which is far too small to
/// show how the clipper scales. Each generator below targets a different cost driver:
/// <see cref="InterlockingCombs"/> produces intersections proportional to the vertex count,
/// <see cref="OverlappingCircles"/> produces almost none, and <see cref="SharedEdgeGrid"/>
/// produces exactly coincident edges, which is the case that pushes the comparers down their
/// expensive paths.
/// </para>
/// </summary>
internal static class SyntheticPolygons
{
    /// <summary>
    /// Two comb shaped polygons whose teeth mesh into one another. Every tooth crosses the
    /// opposing comb's body twice, so the number of edge intersections grows linearly with the
    /// vertex count. This is the case that stresses the sweep line and the event queue.
    /// </summary>
    /// <param name="vertexCount">The approximate combined vertex count of both polygons.</param>
    /// <returns>The subject and clipping polygons.</returns>
    public static (Polygon Subject, Polygon Clipping) InterlockingCombs(int vertexCount)
    {
        // Each comb spends four vertices per tooth and the two combs share the budget.
        int teeth = Math.Max(1, vertexCount / 8);
        const double step = 4D;
        double width = 2D * teeth * step;

        return (UpwardComb(teeth, step, width), DownwardComb(teeth, step, width));
    }

    /// <summary>
    /// Two overlapping circular approximations. They meet in exactly two points, so the vertex
    /// count grows without the intersection count following it. Isolates the cost of the parts
    /// that scale with input size alone: event creation, the heap, and the status line.
    /// </summary>
    /// <param name="vertexCount">The approximate combined vertex count of both polygons.</param>
    /// <returns>The subject and clipping polygons.</returns>
    public static (Polygon Subject, Polygon Clipping) OverlappingCircles(int vertexCount)
    {
        int sides = Math.Max(3, vertexCount / 2);
        return (Circle(sides, 0D, 0D, 100D), Circle(sides, 60D, 0D, 100D));
    }

    /// <summary>
    /// Two grids of square contours, the second shifted by exactly one cell pitch so that most
    /// of its squares land precisely on top of a square of the first. The resulting coincident
    /// edges are what drive <c>SegmentComparer</c> into its collinear branch and leave the
    /// result event list out of order.
    /// </summary>
    /// <param name="vertexCount">The approximate combined vertex count of both polygons.</param>
    /// <returns>The subject and clipping polygons.</returns>
    public static (Polygon Subject, Polygon Clipping) SharedEdgeGrid(int vertexCount)
    {
        // A closed square costs five vertices and the two grids share the budget.
        int cells = Math.Max(1, (int)Math.Sqrt(vertexCount / 10D));
        const double pitch = 2D;

        return (Grid(cells, 0D, pitch), Grid(cells, pitch, pitch));
    }

    /// <summary>
    /// Builds a comb whose body spans y 0..10 with teeth rising to y 25 over the even x slots.
    /// Wound counter clockwise, matching the GeoJSON convention the fixtures follow.
    /// </summary>
    private static Polygon UpwardComb(int teeth, double step, double width)
    {
        Contour contour = new();

        contour.AddVertex(new Vertex(0D, 0D));
        contour.AddVertex(new Vertex(width, 0D));
        contour.AddVertex(new Vertex(width, 10D));

        // Walk the toothed edge from right to left so the ring stays counter clockwise.
        for (int i = teeth - 1; i >= 0; i--)
        {
            double left = 2D * i * step;
            double right = left + step;

            contour.AddVertex(new Vertex(right, 10D));
            contour.AddVertex(new Vertex(right, 25D));
            contour.AddVertex(new Vertex(left, 25D));
            contour.AddVertex(new Vertex(left, 10D));
        }

        return Close(contour);
    }

    /// <summary>
    /// Builds a comb whose body spans y 20..30 with teeth dropping to y 5 over the odd x slots,
    /// so the teeth interleave with those of <see cref="UpwardComb"/> without touching them.
    /// </summary>
    private static Polygon DownwardComb(int teeth, double step, double width)
    {
        Contour contour = new();

        contour.AddVertex(new Vertex(0D, 30D));
        contour.AddVertex(new Vertex(0D, 20D));

        // Walk the toothed edge from left to right so the ring stays counter clockwise.
        for (int i = 0; i < teeth; i++)
        {
            double left = ((2D * i) + 1D) * step;
            double right = left + step;

            contour.AddVertex(new Vertex(left, 20D));
            contour.AddVertex(new Vertex(left, 5D));
            contour.AddVertex(new Vertex(right, 5D));
            contour.AddVertex(new Vertex(right, 20D));
        }

        // The final tooth already ends at (width, 20), so only the right wall is left to add.
        contour.AddVertex(new Vertex(width, 30D));

        return Close(contour);
    }

    private static Polygon Circle(int sides, double centerX, double centerY, double radius)
    {
        Contour contour = new();
        for (int i = 0; i < sides; i++)
        {
            double angle = 2D * Math.PI * i / sides;
            contour.AddVertex(new Vertex(
                centerX + (radius * Math.Cos(angle)),
                centerY + (radius * Math.Sin(angle))));
        }

        return Close(contour);
    }

    private static Polygon Grid(int cells, double offsetX, double pitch)
    {
        // Leave a gap between neighbouring squares so that only the two grids share edges.
        double side = pitch * 0.75D;

        Polygon polygon = new();
        for (int i = 0; i < cells; i++)
        {
            for (int j = 0; j < cells; j++)
            {
                double x = offsetX + (i * pitch);
                double y = j * pitch;

                Contour contour = new();
                contour.AddVertex(new Vertex(x, y));
                contour.AddVertex(new Vertex(x + side, y));
                contour.AddVertex(new Vertex(x + side, y + side));
                contour.AddVertex(new Vertex(x, y + side));

                polygon.Push(CloseRing(contour));
            }
        }

        return polygon;
    }

    /// <summary>
    /// Repeats the first vertex at the end of the contour.
    /// <c>PolygonClipper.Run</c> iterates segments up to <c>VertexCount - 1</c>, so an unclosed
    /// contour silently loses its final edge.
    /// </summary>
    private static Contour CloseRing(Contour contour)
    {
        contour.AddVertex(contour.GetVertex(0));
        return contour;
    }

    private static Polygon Close(Contour contour)
    {
        Polygon polygon = new();
        polygon.Push(CloseRing(contour));
        return polygon;
    }
}
