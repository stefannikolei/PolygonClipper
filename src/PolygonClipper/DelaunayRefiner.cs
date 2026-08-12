// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;

namespace PolygonClipper;

/// <summary>
/// Refines a triangulation toward the constrained Delaunay triangulation by legalizing every
/// interior edge in place with Lawson flips.
/// </summary>
/// <remarks>
/// <para>
/// Adapted from delaunator's edge legalization, as used by the earcut library. Uses non-robust
/// predicates: float input is fine, and the worst case is a not-quite-Delaunay edge, never an
/// invalid mesh.
/// </para>
/// <para>
/// Instances are stateful and hold reusable scratch buffers, so they must not be shared between
/// threads. <see cref="PolygonTriangulator"/> caches one instance per thread.
/// </para>
/// </remarks>
internal sealed class DelaunayRefiner
{
    /// <summary>
    /// The stack of edges pending legalization.
    /// </summary>
    private int[] edgeStack = [];

    /// <summary>
    /// The twin half-edge of each edge, or -1 on the polygon boundary.
    /// </summary>
    private int[] he = [];

    /// <summary>
    /// Open-addressing hash, slot -> half-edge index, valid only if the matching
    /// <see cref="hStamp"/> entry equals <see cref="gen"/>.
    /// </summary>
    private int[] hTable = [];

    /// <summary>
    /// The generation each <see cref="hTable"/> slot was written in.
    /// </summary>
    private uint[] hStamp = [];

    /// <summary>
    /// The pending-in-stack flag of each edge, cleared when the edge is popped.
    /// </summary>
    private byte[] edgeStamp = [];

    /// <summary>
    /// The mask of the power-of-two sized <see cref="hTable"/>.
    /// </summary>
    private int hMask;

    /// <summary>
    /// The current hash generation. Bumping it logically empties the hash.
    /// </summary>
    private uint gen;

    /// <summary>
    /// Refines the given triangulation toward the constrained Delaunay triangulation - maximizing
    /// the minimum angle and removing most slivers.
    /// </summary>
    /// <param name="triangles">The triangle indices, mutated in place.</param>
    /// <param name="coords">The flat vertex coordinates the triangles index into.</param>
    /// <param name="dim">The number of coordinates per vertex in <paramref name="coords"/>.</param>
    public void Refine(Span<int> triangles, ReadOnlySpan<double> coords, int dim)
    {
        Span<int> t = triangles;
        int n = t.Length;
        if (n < 6)
        {
            return;
        }

        this.EnsureScratch(n);

        // Bumping the generation logically empties the hash (no clearing).
        if (this.gen == uint.MaxValue)
        {
            Array.Clear(this.hStamp);
            this.gen = 0;
        }

        this.gen++;
        uint gen = this.gen;
        int[] he = this.he;
        int[] hTable = this.hTable;
        uint[] hStamp = this.hStamp;
        byte[] edgeStamp = this.edgeStamp;
        int[] edgeStack = this.edgeStack;
        int hMask = this.hMask;

        Array.Fill(he, -1, 0, n);

        // Build half-edge twins with an undirected-edge hash; consumed slots mark linked pairs. As
        // each pair is linked we seed the stack with one representative (s, the earlier-inserted
        // edge) - this fuses the initial "push every interior edge" pass into the build, saving a
        // full O(n) scan. edgeStamp is all-zero here (balanced push/pop leaves it clean) and each
        // pair links once, so the seed write needs no dedup guard.
        int i = 0;
        for (int e = 0; e < n; e++)
        {
            int a = t[e], b = t[NextHalfEdge(e)];
            int lo = a < b ? a : b, hi = a < b ? b : a;
            int h = unchecked(((lo * (int)0x9e3779b1) ^ (hi * (int)0x85ebca6b)) & hMask);
            while (hStamp[h] == gen)
            {
                int s = hTable[h];

                // s == -1 marks a consumed slot (a pair already linked) - skip past it.
                if (s != -1)
                {
                    int sa = t[s], sb = t[NextHalfEdge(s)];
                    if ((sa == lo && sb == hi) || (sa == hi && sb == lo))
                    {
                        // Link, then consume the slot.
                        he[e] = s;
                        he[s] = e;
                        hTable[h] = -1;

                        // Seed the interior edge for the cascade.
                        edgeStamp[s] = 1;
                        edgeStack[i++] = s;
                        break;
                    }
                }

                h = (h + 1) & hMask;
            }

            // First occurrence: insert.
            if (hStamp[h] != gen)
            {
                hTable[h] = e;
                hStamp[h] = gen;
            }
        }

        while (i > 0)
        {
            int a = edgeStack[--i];
            edgeStamp[a] = 0;
            int b = he[a];
            if (b == -1)
            {
                continue;
            }

            int a0 = a - (a % 3);
            int b0 = b - (b % 3);
            int ar = a0 + ((a + 2) % 3);
            int al = a0 + ((a + 1) % 3);
            int bl = b0 + ((b + 2) % 3);
            int br = b0 + ((b + 1) % 3);
            int p0 = t[ar], pr = t[a], pl = t[al], p1 = t[bl];

            double x0 = coords[p0 * dim], y0 = coords[(p0 * dim) + 1];
            double xr = coords[pr * dim], yr = coords[(pr * dim) + 1];
            double xl = coords[pl * dim], yl = coords[(pl * dim) + 1];
            double x1 = coords[p1 * dim], y1 = coords[(p1 * dim) + 1];

            // Test InCircle first: most interior edges are already Delaunay (InCircle true -> no
            // flip), so this short-circuits before the two convexity orients on the common path.
            // The quad must also be convex (both new triangles CCW) - flipping a reflex quad would
            // push a triangle outside the polygon. Boundary/hole edges need no guard - they
            // self-protect via he == -1.
            if (!InCircle(x0, y0, xr, yr, xl, yl, x1, y1) &&
                Orient(x0, y0, xr, yr, x1, y1) > 0 && Orient(x0, y0, x1, y1, xl, yl) > 0)
            {
                t[a] = p1;
                t[b] = p0;
                int hbl = he[bl], har = he[ar];
                he[a] = hbl;
                if (hbl != -1)
                {
                    he[hbl] = a;
                }

                he[b] = har;
                if (har != -1)
                {
                    he[har] = b;
                }

                he[ar] = bl;
                he[bl] = ar;

                // Re-check the quad's four outer edges; skip boundary edges (he == -1) and any
                // already queued (edgeStamp), which also keeps the stack bounded by n.
                if (hbl != -1 && edgeStamp[a] == 0)
                {
                    edgeStamp[a] = 1;
                    edgeStack[i++] = a;
                }

                if (har != -1 && edgeStamp[b] == 0)
                {
                    edgeStamp[b] = 1;
                    edgeStack[i++] = b;
                }

                if (he[al] != -1 && edgeStamp[al] == 0)
                {
                    edgeStamp[al] = 1;
                    edgeStack[i++] = al;
                }

                if (he[br] != -1 && edgeStamp[br] == 0)
                {
                    edgeStamp[br] = 1;
                    edgeStack[i++] = br;
                }
            }
        }
    }

    /// <summary>
    /// Calculates the orientation of the triangle (a, b, c).
    /// </summary>
    /// <param name="ax">The x-coordinate of the first vertex.</param>
    /// <param name="ay">The y-coordinate of the first vertex.</param>
    /// <param name="bx">The x-coordinate of the second vertex.</param>
    /// <param name="by">The y-coordinate of the second vertex.</param>
    /// <param name="cx">The x-coordinate of the third vertex.</param>
    /// <param name="cy">The y-coordinate of the third vertex.</param>
    /// <returns>A positive value when the triangle is counterclockwise.</returns>
    private static double Orient(double ax, double ay, double bx, double by, double cx, double cy)
        => ((bx - ax) * (cy - ay)) - ((by - ay) * (cx - ax));

    /// <summary>
    /// Determines whether p is inside or exactly on the circumcircle of the triangle (a, b, c).
    /// </summary>
    /// <remarks>
    /// The sign is negated compared to the usual predicate to match earcut's counterclockwise
    /// winding - the standard sign would build the anti-Delaunay mesh. Cocircular quads are legal
    /// ties, so the refinement only flips when this returns <see langword="false"/>.
    /// </remarks>
    /// <param name="ax">The x-coordinate of the first vertex.</param>
    /// <param name="ay">The y-coordinate of the first vertex.</param>
    /// <param name="bx">The x-coordinate of the second vertex.</param>
    /// <param name="by">The y-coordinate of the second vertex.</param>
    /// <param name="cx">The x-coordinate of the third vertex.</param>
    /// <param name="cy">The y-coordinate of the third vertex.</param>
    /// <param name="px">The x-coordinate of the point to test.</param>
    /// <param name="py">The y-coordinate of the point to test.</param>
    /// <returns><see langword="true"/> if the point is inside or on the circumcircle.</returns>
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

        // A near-cocircular quad is a legal Delaunay tie, but roundoff can flag both an edge and
        // its flip as illegal, cascading into an endless flip loop - so treat a determinant within
        // a small margin of zero as a tie. The determinant's worst-case roundoff error is provably
        // below 9e-16 * (ap + bp + cp)^2 (Shewchuk-style bound), so the margin guarantees every
        // executed flip is illegal in exact arithmetic, and Lawson flipping always terminates.
        double s = ap + bp + cp;
        return (dx * ((ey * cp) - (bp * fy))) - (dy * ((ex * cp) - (bp * fx))) + (ap * ((ex * fy) - (ey * fx)))
            <= 1e-13 * s * s;
    }

    /// <summary>
    /// Gets the next half-edge within the same triangle.
    /// </summary>
    /// <param name="e">The half-edge index.</param>
    /// <returns>The next half-edge index.</returns>
    private static int NextHalfEdge(int e) => e - (e % 3) + ((e + 1) % 3);

    /// <summary>
    /// Grows the scratch arrays on demand.
    /// </summary>
    /// <param name="n">The number of half-edges to accommodate.</param>
    private void EnsureScratch(int n)
    {
        // edgeStack holds at most one entry per half-edge (edgeStamp dedups), so n is a safe cap -
        // sizing it up front lets the cascade push without a bounds/grow check.
        if (this.edgeStack.Length < n)
        {
            this.edgeStack = new int[n];
        }

        if (this.he.Length < n)
        {
            this.he = new int[n];
        }

        if (this.edgeStamp.Length < n)
        {
            this.edgeStamp = new byte[n];
        }

        // Power-of-two table, load factor <= 0.25.
        int size = 1;
        while (size < n * 4)
        {
            size <<= 1;
        }

        if (this.hTable.Length < size)
        {
            this.hTable = new int[size];
            this.hStamp = new uint[size];
        }

        this.hMask = size - 1;
    }
}
