// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace PolygonClipper;

/// <summary>
/// Implements the ear slicing triangulation algorithm used by <see cref="PolygonTriangulator"/>.
/// </summary>
/// <remarks>
/// <para>
/// This is a port of the earcut library by Volodymyr Agafonkin. The algorithm slices ears off a
/// circular doubly linked list of polygon vertices, eliminating holes beforehand by linking them
/// into the outer ring via bridges, and accelerating the containment tests of larger inputs with
/// a z-order (Morton) curve hash.
/// </para>
/// <para>
/// Instances are stateful and hold reusable scratch buffers, so they must not be shared between
/// threads. <see cref="PolygonTriangulator"/> caches one instance per thread.
/// </para>
/// </remarks>
internal sealed class EarcutEngine
{
    /// <summary>
    /// The number of ring edges covered by a single block of the hole bridge index.
    /// </summary>
    private const int K = 16;

    /// <summary>
    /// Pool of nodes reused across calls. Nodes never escape a triangulation, so the pool is
    /// simply rewound at the start of each run instead of being reallocated.
    /// </summary>
    private readonly List<EarcutNode> nodePool = [];

    /// <summary>
    /// The number of nodes handed out from <see cref="nodePool"/> during the current run.
    /// </summary>
    private int nodeCount;

    /// <summary>
    /// Single-vertex holes to preserve through <see cref="FilterPoints(EarcutNode)"/>
    /// (steiner points).
    /// </summary>
    private readonly HashSet<EarcutNode> steiners = [];

    /// <summary>
    /// Set by <see cref="FilterPoints(EarcutNode)"/> whenever it removes at least one node; read
    /// by <see cref="EarcutLinked"/>'s stall handler to decide whether another clip pass is worth
    /// attempting before the costlier stages.
    /// </summary>
    private bool filteredOut;

    /// <summary>
    /// True only while holes are being merged, so that <see cref="RemoveNode"/> keeps the block
    /// index live.
    /// </summary>
    private bool indexActive;

    /// <summary>
    /// The [minX, minY, maxX, maxY] bounding box of each block of the hole bridge index.
    /// </summary>
    private double[] blockBBox = [];

    /// <summary>
    /// The first node of each block's segment.
    /// </summary>
    private EarcutNode[] blockHead = [];

    /// <summary>
    /// The node just past each block's segment (exclusive walk bound).
    /// </summary>
    private EarcutNode[] blockStop = [];

    /// <summary>
    /// The number of blocks currently held by the hole bridge index.
    /// </summary>
    private int numBlocks;

    /// <summary>
    /// Scratch buffers reused across calls and grown on demand: two node reference arrays that
    /// ping-pong during the radix passes, plus parallel z-value arrays so the passes read z from
    /// contiguous memory instead of dereferencing each node. The 256-entry histogram is used for
    /// 8-bit digits; the small histogram keeps per-call setup cheap (most rings are short).
    /// </summary>
    private EarcutNode[] sortArr = [];

    private EarcutNode[] sortBuf = [];
    private uint[] zArr = [];
    private uint[] zBuf = [];
    private readonly uint[] counts = new uint[256];

    /// <summary>
    /// Scratch used to sort the hole queue while preserving the order of equal entries.
    /// </summary>
    private HoleEntry[] holeQueue = [];

    /// <summary>
    /// Triangulates a polygon given as a flat array of vertex coordinates.
    /// </summary>
    /// <param name="data">The flat array of vertex coordinates.</param>
    /// <param name="holeIndices">
    /// The indices (in vertices, not coordinates) where each hole ring starts.
    /// </param>
    /// <param name="dim">The number of coordinates per vertex in <paramref name="data"/>.</param>
    /// <param name="triangles">The collection to append the resulting triangles to.</param>
    public void Triangulate(
        ReadOnlySpan<double> data,
        ReadOnlySpan<int> holeIndices,
        int dim,
        List<int> triangles)
    {
        this.nodeCount = 0;
        this.numBlocks = 0;
        this.indexActive = false;

        bool hasHoles = holeIndices.Length > 0;
        int outerLen = hasHoles ? holeIndices[0] * dim : data.Length;
        if (this.steiners.Count > 0)
        {
            this.steiners.Clear();
        }

        EarcutNode? outerNode = this.LinkedList(data, 0, outerLen, dim, true);

        if (outerNode == null || outerNode.Next == outerNode.Prev)
        {
            return;
        }

        double minX = 0, minY = 0, invSize = 0;

        if (hasHoles)
        {
            outerNode = this.EliminateHoles(data, holeIndices, outerNode, dim);
        }

        // If the shape is not too simple, we'll use z-order curve hash later; calculate polygon bbox.
        if (data.Length > 80 * dim)
        {
            minX = data[0];
            minY = data[1];
            double maxX = minX;
            double maxY = minY;

            for (int i = dim; i < outerLen; i += dim)
            {
                double x = data[i];
                double y = data[i + 1];
                if (x < minX)
                {
                    minX = x;
                }

                if (y < minY)
                {
                    minY = y;
                }

                if (x > maxX)
                {
                    maxX = x;
                }

                if (y > maxY)
                {
                    maxY = y;
                }
            }

            // minX, minY and invSize are later used to transform coords into integers for z-order calculation.
            invSize = Math.Max(maxX - minX, maxY - minY);
            invSize = invSize != 0 ? 32767 / invSize : 0;
        }

        this.EarcutLinked(outerNode, triangles, minX, minY, invSize);
    }

    /// <summary>
    /// Creates a circular doubly linked list from polygon points in the specified winding order.
    /// </summary>
    /// <param name="data">The flat array of vertex coordinates.</param>
    /// <param name="start">The inclusive start offset into <paramref name="data"/>.</param>
    /// <param name="end">The exclusive end offset into <paramref name="data"/>.</param>
    /// <param name="dim">The number of coordinates per vertex in <paramref name="data"/>.</param>
    /// <param name="clockwise">Whether the resulting ring should wind clockwise.</param>
    /// <returns>The last <see cref="EarcutNode"/> of the ring, or <see langword="null"/> when empty.</returns>
    private EarcutNode? LinkedList(ReadOnlySpan<double> data, int start, int end, int dim, bool clockwise)
    {
        EarcutNode? last = null;

        if (clockwise == (PolygonTriangulator.SignedArea(data, start, end, dim) > 0))
        {
            for (int i = start; i < end; i += dim)
            {
                last = this.InsertNode(i / dim, data[i], data[i + 1], last);
            }
        }
        else
        {
            for (int i = end - dim; i >= start; i -= dim)
            {
                last = this.InsertNode(i / dim, data[i], data[i + 1], last);
            }
        }

        if (last != null && PointsEqual(last, last.Next))
        {
            this.RemoveNode(last);
            last = last.Next;
        }

        return last;
    }

    /// <summary>
    /// Removes collinear or coincident points, sweeping the whole ring until nothing is removable.
    /// </summary>
    /// <param name="start">The node to start sweeping from.</param>
    /// <returns>A node of the filtered ring.</returns>
    private EarcutNode FilterPoints(EarcutNode start) => this.FilterPoints(start, start);

    /// <summary>
    /// Removes collinear or coincident points. Removability depends only on a node's immediate
    /// neighbors, so we sweep forward and re-check the predecessor after each removal.
    /// </summary>
    /// <remarks>
    /// When <paramref name="end"/> equals <paramref name="start"/> we sweep the whole ring,
    /// lapping until nothing is removable (the fixpoint the clipper needs). With an explicit
    /// <paramref name="end"/> we heal only the dirty window around a bridge/diagonal cut, stopping
    /// at <paramref name="end"/> rather than lapping, which is O(window) instead of O(ring).
    /// </remarks>
    /// <param name="start">The node to start sweeping from.</param>
    /// <param name="end">The exclusive stop bound of the sweep.</param>
    /// <returns>A node of the filtered ring.</returns>
    private EarcutNode FilterPoints(EarcutNode start, EarcutNode end)
    {
        bool full = ReferenceEquals(end, start);

        EarcutNode p = start;
        bool again;
        do
        {
            again = false;
            if (!ReferenceEquals(p, p.Next) && (this.steiners.Count == 0 || !this.steiners.Contains(p)) &&
                (PointsEqual(p, p.Next) || Area(p.Prev, p, p.Next) == 0))
            {
                if (full || ReferenceEquals(p, end))
                {
                    // Pull the stop bound back past the removal.
                    end = p.Prev;
                }

                this.filteredOut = true;
                this.RemoveNode(p);

                // Re-check the predecessor.
                p = p.Prev;
                again = true;
            }
            else if (full || !ReferenceEquals(p, end))
            {
                p = p.Next;

                // Local heal: keep looping until the sweep reaches end.
                again = !full;
            }
        }
        while (again || !ReferenceEquals(p, end));

        return end;
    }

    /// <summary>
    /// The main ear slicing loop which triangulates a polygon given as a linked list.
    /// </summary>
    /// <param name="ear">The node to start slicing at.</param>
    /// <param name="triangles">The collection to append the resulting triangles to.</param>
    /// <param name="minX">The minimum x-coordinate of the polygon bounding box.</param>
    /// <param name="minY">The minimum y-coordinate of the polygon bounding box.</param>
    /// <param name="invSize">The inverse of the longer side of the polygon bounding box.</param>
    private void EarcutLinked(EarcutNode ear, List<int> triangles, double minX, double minY, double invSize)
    {
        // Interlink polygon nodes in z-order.
        if (invSize != 0)
        {
            this.IndexCurve(ear, minX, minY, invSize);
        }

        EarcutNode stop = ear;
        bool cured = false;

        // Iterate through ears, slicing them one by one.
        while (!ReferenceEquals(ear.Prev, ear.Next))
        {
            EarcutNode prev = ear.Prev;
            EarcutNode next = ear.Next;

            if (Area(prev, ear, next) < 0 && (invSize != 0 ? IsEarHashed(ear, minX, minY, invSize) : IsEar(ear)))
            {
                // Cut off the triangle.
                triangles.Add(prev.Index);
                triangles.Add(ear.Index);
                triangles.Add(next.Index);

                this.RemoveNode(ear);
                ear = next;
                stop = next;
                continue;
            }

            ear = next;

            // If we looped through the whole remaining polygon and can't find any more ears.
            if (ReferenceEquals(ear, stop))
            {
                // Try filtering collinear/coincident points and slicing again - repeat as long as
                // filtering actually removes nodes, since each removal can expose new ears.
                this.filteredOut = false;
                ear = this.FilterPoints(ear);
                if (this.filteredOut)
                {
                    stop = ear;
                    continue;
                }

                // Filtering is exhausted: cure small local self-intersections once, then retry.
                if (!cured)
                {
                    ear = this.CureLocalIntersections(ear, triangles);
                    stop = ear;
                    cured = true;
                    continue;
                }

                // As a last resort, try splitting the remaining polygon into two.
                this.SplitEarcut(ear, triangles, minX, minY, invSize);
                break;
            }
        }
    }

    /// <summary>
    /// Checks whether a polygon node forms a valid ear with adjacent nodes.
    /// </summary>
    /// <param name="ear">The candidate ear.</param>
    /// <returns><see langword="true"/> if the node forms a valid ear; otherwise <see langword="false"/>.</returns>
    private static bool IsEar(EarcutNode ear)
    {
        // The reflex check (area(a, b, c) >= 0) is hoisted into the EarcutLinked caller.
        EarcutNode a = ear.Prev, b = ear, c = ear.Next;
        double ax = a.X, bx = b.X, cx = c.X, ay = a.Y, by = b.Y, cy = c.Y;

        // Triangle bbox.
        double x0 = Math.Min(Math.Min(ax, bx), cx);
        double y0 = Math.Min(Math.Min(ay, by), cy);
        double x1 = Math.Max(Math.Max(ax, bx), cx);
        double y1 = Math.Max(Math.Max(ay, by), cy);

        // Make sure we don't have other points inside the potential ear.
        EarcutNode p = c.Next;
        while (!ReferenceEquals(p, a))
        {
            if (p.X >= x0 && p.X <= x1 && p.Y >= y0 && p.Y <= y1 && !(ax == p.X && ay == p.Y) &&
                PointInTriangle(ax, ay, bx, by, cx, cy, p.X, p.Y) && Area(p.Prev, p, p.Next) >= 0)
            {
                return false;
            }

            p = p.Next;
        }

        return true;
    }

    /// <summary>
    /// Checks whether a polygon node forms a valid ear with adjacent nodes, using the z-order
    /// curve hash to limit the number of containment tests.
    /// </summary>
    /// <param name="ear">The candidate ear.</param>
    /// <param name="minX">The minimum x-coordinate of the polygon bounding box.</param>
    /// <param name="minY">The minimum y-coordinate of the polygon bounding box.</param>
    /// <param name="invSize">The inverse of the longer side of the polygon bounding box.</param>
    /// <returns><see langword="true"/> if the node forms a valid ear; otherwise <see langword="false"/>.</returns>
    private static bool IsEarHashed(EarcutNode ear, double minX, double minY, double invSize)
    {
        // The reflex check is hoisted into the EarcutLinked caller (see IsEar).
        EarcutNode a = ear.Prev, b = ear, c = ear.Next;
        double ax = a.X, bx = b.X, cx = c.X, ay = a.Y, by = b.Y, cy = c.Y;

        // Triangle bbox.
        double x0 = Math.Min(Math.Min(ax, bx), cx);
        double y0 = Math.Min(Math.Min(ay, by), cy);
        double x1 = Math.Max(Math.Max(ax, bx), cx);
        double y1 = Math.Max(Math.Max(ay, by), cy);

        // z-order range for the current triangle bbox.
        int minZ = ZOrder(x0, y0, minX, minY, invSize);
        int maxZ = ZOrder(x1, y1, minX, minY, invSize);

        // Look for points inside the triangle in decreasing z-order.
        EarcutNode? p = ear.PrevZ;
        while (p != null && p.Z >= minZ)
        {
            if (p.X >= x0 && p.X <= x1 && p.Y >= y0 && p.Y <= y1 && !ReferenceEquals(p, c) && !(ax == p.X && ay == p.Y) &&
                PointInTriangle(ax, ay, bx, by, cx, cy, p.X, p.Y) && Area(p.Prev, p, p.Next) >= 0)
            {
                return false;
            }

            p = p.PrevZ;
        }

        // Look for points in increasing z-order.
        EarcutNode? n = ear.NextZ;
        while (n != null && n.Z <= maxZ)
        {
            if (n.X >= x0 && n.X <= x1 && n.Y >= y0 && n.Y <= y1 && !ReferenceEquals(n, c) && !(ax == n.X && ay == n.Y) &&
                PointInTriangle(ax, ay, bx, by, cx, cy, n.X, n.Y) && Area(n.Prev, n, n.Next) >= 0)
            {
                return false;
            }

            n = n.NextZ;
        }

        return true;
    }

    /// <summary>
    /// Goes through all polygon nodes and cures small local self-intersections.
    /// </summary>
    /// <param name="start">The node to start at.</param>
    /// <param name="triangles">The collection to append the resulting triangles to.</param>
    /// <returns>A node of the remaining ring.</returns>
    private EarcutNode CureLocalIntersections(EarcutNode start, List<int> triangles)
    {
        EarcutNode p = start;
        bool cured = false;
        do
        {
            EarcutNode a = p.Prev;
            EarcutNode b = p.Next.Next;

            if (Intersects(a, p, p.Next, b, false) && LocallyInside(a, b) && LocallyInside(b, a))
            {
                triangles.Add(a.Index);
                triangles.Add(p.Index);
                triangles.Add(b.Index);

                // Remove the two nodes involved.
                this.RemoveNode(p);
                this.RemoveNode(p.Next);

                p = start = b;
                cured = true;
            }

            p = p.Next;
        }
        while (!ReferenceEquals(p, start));

        return cured ? this.FilterPoints(p) : p;
    }

    /// <summary>
    /// Tries splitting the polygon into two and triangulates them independently.
    /// </summary>
    /// <param name="start">The node to start at.</param>
    /// <param name="triangles">The collection to append the resulting triangles to.</param>
    /// <param name="minX">The minimum x-coordinate of the polygon bounding box.</param>
    /// <param name="minY">The minimum y-coordinate of the polygon bounding box.</param>
    /// <param name="invSize">The inverse of the longer side of the polygon bounding box.</param>
    private void SplitEarcut(EarcutNode start, List<int> triangles, double minX, double minY, double invSize)
    {
        // Look for a valid diagonal that divides the polygon into two.
        EarcutNode a = start;
        do
        {
            EarcutNode b = a.Next.Next;
            while (!ReferenceEquals(b, a.Prev))
            {
                if (a.Index != b.Index && IsValidDiagonal(a, b))
                {
                    // Split the polygon in two by the diagonal.
                    EarcutNode c = this.SplitPolygon(a, b);

                    // Filter collinear points around the cuts.
                    a = this.FilterPoints(a, a.Next);
                    c = this.FilterPoints(c, c.Next);

                    // Run earcut on each half.
                    this.EarcutLinked(a, triangles, minX, minY, invSize);
                    this.EarcutLinked(c, triangles, minX, minY, invSize);
                    return;
                }

                b = b.Next;
            }

            a = a.Next;
        }
        while (!ReferenceEquals(a, start));
    }

    /// <summary>
    /// Links every hole into the outer loop, producing a single-ring polygon without holes.
    /// </summary>
    /// <param name="data">The flat array of vertex coordinates.</param>
    /// <param name="holeIndices">The indices where each hole ring starts.</param>
    /// <param name="outerNode">A node of the outer ring.</param>
    /// <param name="dim">The number of coordinates per vertex in <paramref name="data"/>.</param>
    /// <returns>A node of the merged ring.</returns>
    private EarcutNode EliminateHoles(
        ReadOnlySpan<double> data,
        ReadOnlySpan<int> holeIndices,
        EarcutNode outerNode,
        int dim)
    {
        int len = holeIndices.Length;
        if (this.holeQueue.Length < len)
        {
            this.holeQueue = new HoleEntry[len];
        }

        HoleEntry[] queue = this.holeQueue;
        int queueCount = 0;

        for (int i = 0; i < len; i++)
        {
            int start = holeIndices[i] * dim;
            int end = i < len - 1 ? holeIndices[i + 1] * dim : data.Length;
            EarcutNode list = this.LinkedList(data, start, end, dim, false)!;
            if (ReferenceEquals(list, list.Next))
            {
                this.steiners.Add(list);
            }

            queue[queueCount] = new HoleEntry(GetLeftmost(list), queueCount);
            queueCount++;
        }

        // Array.Sort is unstable, so the insertion order is used as the final tie breaker to match
        // the stable sort the reference implementation relies on.
        Array.Sort(queue, 0, queueCount, HoleEntryComparer.Instance);

        // Block-bbox index for FindHoleBridge, grown append-only as holes merge (see notes above
        // BuildBlockIndex). Seed it with the outer ring, then append each merged hole.
        this.BuildBlockIndex(data.Length / dim, len);
        this.IndexSegment(outerNode, outerNode);

        // Process holes from left to right; indexActive lets RemoveNode keep block bboxes live as
        // FilterPoints heals edges during merges (see GrowBlock).
        this.indexActive = true;
        for (int i = 0; i < queueCount; i++)
        {
            outerNode = this.EliminateHole(queue[i].Node, outerNode);
        }

        this.indexActive = false;

        // Collapse collinear/coincident points across the whole merged ring once before clipping.
        return this.FilterPoints(outerNode);
    }

    /// <summary>
    /// Compares two hole entries by leftmost point, then by the slope of the outgoing edge.
    /// </summary>
    /// <remarks>
    /// When the leftmost point of 2 holes meet at a vertex, sort the holes counterclockwise so
    /// that the bridge to the outer shell is always the point that they meet at.
    /// </remarks>
    /// <param name="a">The first node.</param>
    /// <param name="b">The second node.</param>
    /// <returns>A signed value indicating the relative order of the nodes.</returns>
    private static int CompareXYSlope(EarcutNode a, EarcutNode b)
    {
        int sign = Sign(a.X - b.X);
        if (sign != 0)
        {
            return sign;
        }

        sign = Sign(a.Y - b.Y);
        if (sign != 0)
        {
            return sign;
        }

        return Sign(((a.Next.Y - a.Y) / (a.Next.X - a.X)) -
                    ((b.Next.Y - b.Y) / (b.Next.X - b.X)));
    }

    /// <summary>
    /// Returns the sign of the given value, treating NaN as zero.
    /// </summary>
    /// <remarks>
    /// The slope terms of <see cref="CompareXYSlope"/> are infinite for vertical edges and their
    /// difference is therefore NaN when both edges are vertical. The reference implementation
    /// relies on such a comparison being treated as equal.
    /// </remarks>
    /// <param name="value">The value to inspect.</param>
    /// <returns>-1, 0 or 1.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int Sign(double value)
    {
        if (value < 0)
        {
            return -1;
        }

        if (value > 0)
        {
            return 1;
        }

        return 0;
    }

    /// <summary>
    /// Finds a bridge between vertices that connects a hole with an outer ring and links it.
    /// </summary>
    /// <param name="hole">The leftmost node of the hole.</param>
    /// <param name="outerNode">A node of the outer ring.</param>
    /// <returns>A node of the merged ring.</returns>
    private EarcutNode EliminateHole(EarcutNode hole, EarcutNode outerNode)
    {
        EarcutNode? bridge = this.FindHoleBridge(hole, outerNode);
        if (bridge == null)
        {
            return outerNode;
        }

        EarcutNode bridgeReverse = this.SplitPolygon(bridge, hole);

        // Index the merged-in segment before filtering: in ring order the splice runs
        // bridge -> hole -> bridgeReverse -> bridge2 -> (bridge's old next), covering the hole's
        // edges and both new slit edges. FilterPoints below only drops collinear/coincident
        // points, so these bboxes stay valid (conservative) supersets.
        EarcutNode bridge2 = bridgeReverse.Next;
        this.IndexSegment(bridge, bridge2.Next);

        // Heal collinear/coincident points around the two new slit edges.
        this.FilterPoints(bridgeReverse, bridgeReverse.Next);
        return this.FilterPoints(bridge, bridge.Next);
    }

    /// <summary>
    /// Sizes the block-bbox index used by <see cref="FindHoleBridge"/> for the current run.
    /// </summary>
    /// <remarks>
    /// <para>
    /// The index holds one [minX, minY, maxX, maxY] bbox per <see cref="K"/> consecutive ring
    /// edges, so the leftward-ray scan can skip whole blocks in O(1) instead of walking the entire
    /// merged ring. It is grown append-only: the outer ring seeds it, then each merged hole appends
    /// a segment (head node, stop node, K-blocks over head..stop); independent segments, not a ring
    /// tiling, since splices land mid-ring. Buffers are sized once from the input upper bound and
    /// reused across calls.
    /// </para>
    /// <para>
    /// <see cref="FilterPoints(EarcutNode)"/> only drops collinear/coincident points, so a stale
    /// bbox stays a conservative superset of its live edges (never a false skip); the scan skips
    /// dead nodes and lazily advances a dead stop. Blocks are scanned in append (not ring) order,
    /// so the chosen bridge can differ from the un-indexed code - a different but equally valid
    /// result.
    /// </para>
    /// </remarks>
    /// <param name="maxNodes">The number of vertices in the input.</param>
    /// <param name="numHoles">The number of holes in the input.</param>
    private void BuildBlockIndex(int maxNodes, int numHoles)
    {
        // Upper bound: every input node indexed once, +2 bridge nodes per hole, plus a partial
        // trailing block per appended segment (outer ring + one per hole).
        int maxBlocks = (int)Math.Ceiling((maxNodes + (2 * numHoles)) / (double)K) + numHoles + 2;
        if (this.blockBBox.Length < maxBlocks * 4)
        {
            this.blockBBox = new double[maxBlocks * 4];
        }

        if (this.blockHead.Length < maxBlocks)
        {
            this.blockHead = new EarcutNode[maxBlocks];
            this.blockStop = new EarcutNode[maxBlocks];
        }

        this.numBlocks = 0;
    }

    /// <summary>
    /// Indexes the ring run head..stop (exclusive) as ceil(len / K) blocks. When
    /// <paramref name="head"/> equals <paramref name="stop"/> the whole ring is indexed. Each
    /// block's bbox covers both endpoints of every edge it owns.
    /// </summary>
    /// <param name="head">The first node of the run.</param>
    /// <param name="stop">The node just past the run.</param>
    private void IndexSegment(EarcutNode head, EarcutNode stop)
    {
        double[] blockBBox = this.blockBBox;
        EarcutNode p = head;
        do
        {
            int b = this.numBlocks++;
            this.blockHead[b] = p;
            double minX = p.X, minY = p.Y, maxX = p.X, maxY = p.Y;
            int k = 0;
            do
            {
                // Edge p->c; bbox must bound both endpoints.
                EarcutNode c = p.Next;

                // Reuse z as the owning block during hole elimination (see GrowBlock).
                p.Z = b;
                if (c.X < minX)
                {
                    minX = c.X;
                }

                if (c.X > maxX)
                {
                    maxX = c.X;
                }

                if (c.Y < minY)
                {
                    minY = c.Y;
                }

                if (c.Y > maxY)
                {
                    maxY = c.Y;
                }

                p = c;
            }
            while (++k < K && !ReferenceEquals(p, stop));

            this.blockStop[b] = p;
            int g = b * 4;
            blockBBox[g] = minX;
            blockBBox[g + 1] = minY;
            blockBBox[g + 2] = maxX;
            blockBBox[g + 3] = maxY;
        }
        while (!ReferenceEquals(p, stop));
    }

    /// <summary>
    /// When <see cref="FilterPoints(EarcutNode)"/> heals an edge head->tail (removing the collinear
    /// node between them), the healed edge can extend past head's frozen block bbox if its old far
    /// endpoint lived in another block; grow head's block bbox to cover tail so the leftward-ray
    /// prune can't false-skip it.
    /// </summary>
    /// <param name="head">The node the healed edge starts at.</param>
    /// <param name="tail">The node the healed edge ends at.</param>
    private void GrowBlock(EarcutNode head, EarcutNode tail)
    {
        double[] blockBBox = this.blockBBox;
        int g = head.Z * 4;
        if (tail.X < blockBBox[g])
        {
            blockBBox[g] = tail.X;
        }

        if (tail.Y < blockBBox[g + 1])
        {
            blockBBox[g + 1] = tail.Y;
        }

        if (tail.X > blockBBox[g + 2])
        {
            blockBBox[g + 2] = tail.X;
        }

        if (tail.Y > blockBBox[g + 3])
        {
            blockBBox[g + 3] = tail.Y;
        }
    }

    /// <summary>
    /// Gets the block's exclusive walk bound, advancing it past nodes that are no longer part of
    /// the ring.
    /// </summary>
    /// <param name="b">The block index.</param>
    /// <returns>The live stop node.</returns>
    private EarcutNode LiveBlockStop(int b)
    {
        EarcutNode stop = this.blockStop[b];
        while (!ReferenceEquals(stop.Prev.Next, stop))
        {
            stop = stop.Next;
        }

        this.blockStop[b] = stop;
        return stop;
    }

    /// <summary>
    /// Gets the block's head node, advancing it past nodes that are no longer part of the ring.
    /// </summary>
    /// <remarks>
    /// The block's head node can be removed by <see cref="FilterPoints(EarcutNode)"/> during
    /// merges; advance it to the next live node so the walk doesn't start on (and immediately
    /// terminate at) a dead node. For the single full-ring seed block (head == stop) the same
    /// forward advance keeps them equal, so the do-while still laps the whole ring instead of
    /// collapsing to an empty walk.
    /// </remarks>
    /// <param name="b">The block index.</param>
    /// <returns>The live head node.</returns>
    private EarcutNode LiveBlockHead(int b)
    {
        EarcutNode head = this.blockHead[b];
        while (!ReferenceEquals(head.Prev.Next, head))
        {
            head = head.Next;
        }

        this.blockHead[b] = head;
        return head;
    }

    /// <summary>
    /// David Eberly's algorithm for finding a bridge between a hole and an outer polygon.
    /// </summary>
    /// <param name="hole">The leftmost node of the hole.</param>
    /// <param name="outerNode">A node of the outer ring.</param>
    /// <returns>The bridge node, or <see langword="null"/> if none was found.</returns>
    private EarcutNode? FindHoleBridge(EarcutNode hole, EarcutNode outerNode)
    {
        double[] blockBBox = this.blockBBox;
        EarcutNode p = outerNode;
        double hx = hole.X;
        double hy = hole.Y;
        double qx = double.NegativeInfinity;
        EarcutNode? m = null;

        // Find a segment intersected by a ray from the hole's leftmost point to the left; the
        // segment's endpoint with lesser x will be the potential connection point, unless they
        // intersect at a vertex, then choose the vertex.
        if (PointsEqual(hole, p))
        {
            return p;
        }

        // Scan blocks; skip any whose bbox can't hold a crossing that beats qx and lies left of hx
        // (the prune Morton order can't express - explicit per-axis [minY,maxY]/[minX,maxX]).
        for (int b = 0, g = 0; b < this.numBlocks; b++, g += 4)
        {
            if (hy < blockBBox[g + 1] || hy > blockBBox[g + 3] || blockBBox[g] > hx || blockBBox[g + 2] <= qx)
            {
                continue;
            }

            // Ensure the walk's exclusive bound is live so we don't overrun into other blocks.
            EarcutNode stop = this.LiveBlockStop(b);

            p = this.LiveBlockHead(b);
            do
            {
                // Skip nodes removed by FilterPoints (stale in the index).
                if (ReferenceEquals(p.Prev.Next, p))
                {
                    if (PointsEqual(hole, p.Next))
                    {
                        return p.Next;
                    }
                    else if (hy <= p.Y && hy >= p.Next.Y && p.Next.Y != p.Y)
                    {
                        double x = p.X + ((hy - p.Y) * (p.Next.X - p.X) / (p.Next.Y - p.Y));
                        if (x <= hx && x > qx)
                        {
                            qx = x;
                            m = p.X < p.Next.X ? p : p.Next;
                            if (x == hx)
                            {
                                // The hole touches the outer segment; pick the leftmost endpoint.
                                return m;
                            }
                        }
                    }
                }

                p = p.Next;
            }
            while (!ReferenceEquals(p, stop));
        }

        if (m == null)
        {
            return null;
        }

        // Look for points inside the triangle of hole point, segment intersection and endpoint;
        // if there are no points found, we have a valid connection; otherwise choose the point of
        // the minimum angle with the ray as connection point.
        double mx = m.X;
        double my = m.Y;

        // The triangle's y span; the x span is [mx, hx].
        double tminY = Math.Min(hy, my);
        double tmaxY = Math.Max(hy, my);
        double tanMin = double.PositiveInfinity;

        // Scan the same blocks; skip any whose bbox can't overlap the triangle's
        // [mx,hx]x[tminY,tmaxY] box.
        for (int b = 0, g = 0; b < this.numBlocks; b++, g += 4)
        {
            if (blockBBox[g + 2] < mx || blockBBox[g] > hx || blockBBox[g + 3] < tminY || blockBBox[g + 1] > tmaxY)
            {
                continue;
            }

            EarcutNode stop = this.LiveBlockStop(b);

            p = this.LiveBlockHead(b);
            do
            {
                // Skip dead nodes.
                if (ReferenceEquals(p.Prev.Next, p) && hx >= p.X && p.X >= mx && hx != p.X &&
                    PointInTriangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy, p.X, p.Y))
                {
                    // Tangential.
                    double tan = Math.Abs(hy - p.Y) / (hx - p.X);

                    // If the hole point sits on p's horizontal edge (T-junction touch): the bridge
                    // runs along that edge - LocallyInside rejects it as collinear, but it's valid.
                    if ((LocallyInside(p, hole) || (p.Y == hy && p.Next.Y == hy && p.Next.X > hx)) &&
                        (tan < tanMin || (tan == tanMin && (p.X > m.X || (p.X == m.X && SectorContainsSector(m, p))))))
                    {
                        m = p;
                        tanMin = tan;
                    }
                }

                p = p.Next;
            }
            while (!ReferenceEquals(p, stop));
        }

        return m;
    }

    /// <summary>
    /// Determines whether the sector in vertex m contains the sector in vertex p in the same
    /// coordinates.
    /// </summary>
    /// <param name="m">The first node.</param>
    /// <param name="p">The second node.</param>
    /// <returns><see langword="true"/> if the sector contains the other; otherwise <see langword="false"/>.</returns>
    private static bool SectorContainsSector(EarcutNode m, EarcutNode p)
        => Area(m.Prev, m, p.Prev) < 0 && Area(p.Next, m, m.Next) < 0;

    /// <summary>
    /// Interlinks polygon nodes in z-order: collect into an array, sort by z, relink.
    /// </summary>
    /// <param name="start">The node to start at.</param>
    /// <param name="minX">The minimum x-coordinate of the polygon bounding box.</param>
    /// <param name="minY">The minimum y-coordinate of the polygon bounding box.</param>
    /// <param name="invSize">The inverse of the longer side of the polygon bounding box.</param>
    private void IndexCurve(EarcutNode start, double minX, double minY, double invSize)
    {
        EarcutNode p = start;
        int n = 0;
        do
        {
            // Always (re)compute: z may still hold a block index left over from hole elimination.
            p.Z = ZOrder(p.X, p.Y, minX, minY, invSize);
            if (this.sortArr.Length <= n)
            {
                Array.Resize(ref this.sortArr, Math.Max(16, this.sortArr.Length * 2));
            }

            this.sortArr[n++] = p;
            p = p.Next;
        }
        while (!ReferenceEquals(p, start));

        this.SortNodes(n);

        EarcutNode? prev = null;
        for (int i = 0; i < n; i++)
        {
            EarcutNode node = this.sortArr[i];
            node.PrevZ = prev;
            if (prev != null)
            {
                prev.NextZ = node;
            }

            prev = node;
        }

        prev!.NextZ = null;
    }

    /// <summary>
    /// Sorts the first n nodes of the scratch array by z, in place: insertion sort for small n
    /// (cheaper than histogram setup), else LSD radix in four 8-bit passes (covering z's 30 bits).
    /// </summary>
    /// <param name="n">The number of nodes to sort.</param>
    private void SortNodes(int n)
    {
        EarcutNode[] sortArr = this.sortArr;
        if (n <= 32)
        {
            for (int i = 1; i < n; i++)
            {
                EarcutNode node = sortArr[i];
                int z = node.Z;
                int j = i - 1;
                while (j >= 0 && sortArr[j].Z > z)
                {
                    sortArr[j + 1] = sortArr[j];
                    j--;
                }

                sortArr[j + 1] = node;
            }

            return;
        }

        if (this.zArr.Length < n)
        {
            this.zArr = new uint[n];
            this.zBuf = new uint[n];
            this.sortBuf = new EarcutNode[n];
        }

        uint[] zArr = this.zArr;
        for (int i = 0; i < n; i++)
        {
            zArr[i] = (uint)sortArr[i].Z;
        }

        // An even pass count lands the sorted result back in sortArr.
        this.RadixPass(n, sortArr, this.zArr, this.sortBuf, this.zBuf, 0);
        this.RadixPass(n, this.sortBuf, this.zBuf, sortArr, this.zArr, 8);
        this.RadixPass(n, sortArr, this.zArr, this.sortBuf, this.zBuf, 16);
        this.RadixPass(n, this.sortBuf, this.zBuf, sortArr, this.zArr, 24);
    }

    /// <summary>
    /// Performs one LSD radix pass: stably scatter the first n nodes (and their z) from src to dst,
    /// bucketed by the 8-bit digit of z at the given bit shift.
    /// </summary>
    /// <param name="n">The number of nodes to scatter.</param>
    /// <param name="src">The source nodes.</param>
    /// <param name="srcZ">The source z-values.</param>
    /// <param name="dst">The destination nodes.</param>
    /// <param name="dstZ">The destination z-values.</param>
    /// <param name="shift">The bit shift of the digit to bucket by.</param>
    private void RadixPass(int n, EarcutNode[] src, uint[] srcZ, EarcutNode[] dst, uint[] dstZ, int shift)
    {
        uint[] counts = this.counts;
        Array.Clear(counts);
        for (int i = 0; i < n; i++)
        {
            counts[(srcZ[i] >> shift) & 0xff]++;
        }

        // Turn per-bucket counts into start offsets (prefix sum).
        uint sum = 0;
        for (int b = 0; b < 256; b++)
        {
            uint c = counts[b];
            counts[b] = sum;
            sum += c;
        }

        for (int i = 0; i < n; i++)
        {
            uint z = srcZ[i];
            uint pos = counts[(z >> shift) & 0xff]++;
            dst[pos] = src[i];
            dstZ[pos] = z;
        }
    }

    /// <summary>
    /// Calculates the z-order of a point given its coordinates and the inverse of the longer side
    /// of the data bounding box.
    /// </summary>
    /// <param name="x">The x-coordinate.</param>
    /// <param name="y">The y-coordinate.</param>
    /// <param name="minX">The minimum x-coordinate of the polygon bounding box.</param>
    /// <param name="minY">The minimum y-coordinate of the polygon bounding box.</param>
    /// <param name="invSize">The inverse of the longer side of the polygon bounding box.</param>
    /// <returns>The z-order curve value.</returns>
    private static int ZOrder(double x, double y, double minX, double minY, double invSize)
    {
        // Coords are transformed into the non-negative 15-bit integer range.
        int ix = (int)((x - minX) * invSize);
        int iy = (int)((y - minY) * invSize);

        ix = (ix | (ix << 8)) & 0x00FF00FF;
        ix = (ix | (ix << 4)) & 0x0F0F0F0F;
        ix = (ix | (ix << 2)) & 0x33333333;
        ix = (ix | (ix << 1)) & 0x55555555;

        iy = (iy | (iy << 8)) & 0x00FF00FF;
        iy = (iy | (iy << 4)) & 0x0F0F0F0F;
        iy = (iy | (iy << 2)) & 0x33333333;
        iy = (iy | (iy << 1)) & 0x55555555;

        return ix | (iy << 1);
    }

    /// <summary>
    /// Finds the leftmost node of a polygon ring.
    /// </summary>
    /// <param name="start">The node to start at.</param>
    /// <returns>The leftmost node.</returns>
    private static EarcutNode GetLeftmost(EarcutNode start)
    {
        EarcutNode p = start;
        EarcutNode leftmost = start;
        do
        {
            if (p.X < leftmost.X || (p.X == leftmost.X && p.Y < leftmost.Y))
            {
                leftmost = p;
            }

            p = p.Next;
        }
        while (!ReferenceEquals(p, start));

        return leftmost;
    }

    /// <summary>
    /// Determines whether a point lies within a convex triangle.
    /// </summary>
    /// <param name="ax">The x-coordinate of the first triangle vertex.</param>
    /// <param name="ay">The y-coordinate of the first triangle vertex.</param>
    /// <param name="bx">The x-coordinate of the second triangle vertex.</param>
    /// <param name="by">The y-coordinate of the second triangle vertex.</param>
    /// <param name="cx">The x-coordinate of the third triangle vertex.</param>
    /// <param name="cy">The y-coordinate of the third triangle vertex.</param>
    /// <param name="px">The x-coordinate of the point.</param>
    /// <param name="py">The y-coordinate of the point.</param>
    /// <returns><see langword="true"/> if the point lies within the triangle; otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool PointInTriangle(
        double ax,
        double ay,
        double bx,
        double by,
        double cx,
        double cy,
        double px,
        double py)
        => ((cx - px) * (ay - py) >= (ax - px) * (cy - py)) &&
           ((ax - px) * (by - py) >= (bx - px) * (ay - py)) &&
           ((bx - px) * (cy - py) >= (cx - px) * (by - py));

    /// <summary>
    /// Determines whether a diagonal between two polygon nodes is valid (lies in the polygon
    /// interior).
    /// </summary>
    /// <param name="a">The first node.</param>
    /// <param name="b">The second node.</param>
    /// <returns><see langword="true"/> when the diagonal is valid; otherwise <see langword="false"/>.</returns>
    private static bool IsValidDiagonal(EarcutNode a, EarcutNode b)
    {
        // Degenerate case.
        bool zeroLength = PointsEqual(a, b) && Area(a.Prev, a, a.Next) > 0 && Area(b.Prev, b, b.Next) > 0;

        return a.Next.Index != b.Index &&

            // Locally visible.
            (zeroLength || (LocallyInside(a, b) && LocallyInside(b, a) &&

            // No opposite-facing sectors.
            (Area(a.Prev, a, b.Prev) != 0 || Area(a, b.Prev, b) != 0))) &&

            // Doesn't intersect other edges, diagonal inside polygon.
            !IntersectsPolygon(a, b) && (zeroLength || MiddleInside(a, b));
    }

    /// <summary>
    /// Calculates the signed area of a triangle.
    /// </summary>
    /// <param name="p">The first node.</param>
    /// <param name="q">The second node.</param>
    /// <param name="r">The third node.</param>
    /// <returns>The signed area.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double Area(EarcutNode p, EarcutNode q, EarcutNode r)
        => ((q.Y - p.Y) * (r.X - q.X)) - ((q.X - p.X) * (r.Y - q.Y));

    /// <summary>
    /// Determines whether two nodes represent the same point.
    /// </summary>
    /// <param name="p1">The first node.</param>
    /// <param name="p2">The second node.</param>
    /// <returns><see langword="true"/> if the points are equal; otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool PointsEqual(EarcutNode p1, EarcutNode p2)
        => p1.X == p2.X && p1.Y == p2.Y;

    /// <summary>
    /// Determines whether two segments intersect, including collinear boundary touches.
    /// </summary>
    /// <param name="p1">The start of the first segment.</param>
    /// <param name="q1">The end of the first segment.</param>
    /// <param name="p2">The start of the second segment.</param>
    /// <param name="q2">The end of the second segment.</param>
    /// <returns><see langword="true"/> if the segments intersect; otherwise <see langword="false"/>.</returns>
    private static bool Intersects(EarcutNode p1, EarcutNode q1, EarcutNode p2, EarcutNode q2)
        => Intersects(p1, q1, p2, q2, true);

    /// <summary>
    /// Determines whether two segments intersect.
    /// </summary>
    /// <param name="p1">The start of the first segment.</param>
    /// <param name="q1">The end of the first segment.</param>
    /// <param name="p2">The start of the second segment.</param>
    /// <param name="q2">The end of the second segment.</param>
    /// <param name="includeBoundary">Whether collinear boundary touches count as intersections.</param>
    /// <returns><see langword="true"/> if the segments intersect; otherwise <see langword="false"/>.</returns>
    private static bool Intersects(EarcutNode p1, EarcutNode q1, EarcutNode p2, EarcutNode q2, bool includeBoundary)
    {
        double o1 = Area(p1, q1, p2);
        double o2 = Area(p1, q1, q2);
        double o3 = Area(p2, q2, p1);
        double o4 = Area(p2, q2, q1);

        if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0)))
        {
            return true;
        }

        if (!includeBoundary)
        {
            return false;
        }

        // p1, q1 and p2 are collinear and p2 lies on p1q1.
        if (o1 == 0 && OnSegment(p1, p2, q1))
        {
            return true;
        }

        // p1, q1 and q2 are collinear and q2 lies on p1q1.
        if (o2 == 0 && OnSegment(p1, q2, q1))
        {
            return true;
        }

        // p2, q2 and p1 are collinear and p1 lies on p2q2.
        if (o3 == 0 && OnSegment(p2, p1, q2))
        {
            return true;
        }

        // p2, q2 and q1 are collinear and q1 lies on p2q2.
        if (o4 == 0 && OnSegment(p2, q1, q2))
        {
            return true;
        }

        return false;
    }

    /// <summary>
    /// For collinear points p, q, r, determines whether the point q lies on the segment pr.
    /// </summary>
    /// <param name="p">The start of the segment.</param>
    /// <param name="q">The point to test.</param>
    /// <param name="r">The end of the segment.</param>
    /// <returns><see langword="true"/> if the point lies on the segment; otherwise <see langword="false"/>.</returns>
    private static bool OnSegment(EarcutNode p, EarcutNode q, EarcutNode r)
        => q.X <= Math.Max(p.X, r.X) && q.X >= Math.Min(p.X, r.X) &&
           q.Y <= Math.Max(p.Y, r.Y) && q.Y >= Math.Min(p.Y, r.Y);

    /// <summary>
    /// Determines whether a polygon diagonal intersects any polygon segments.
    /// </summary>
    /// <param name="a">The first node of the diagonal.</param>
    /// <param name="b">The second node of the diagonal.</param>
    /// <returns><see langword="true"/> if the diagonal intersects a segment; otherwise <see langword="false"/>.</returns>
    private static bool IntersectsPolygon(EarcutNode a, EarcutNode b)
    {
        // Diagonal bbox; an edge whose bbox can't overlap it can't intersect it, so skip the
        // orientation test for those (the common case - the diagonal is short).
        double minX = Math.Min(a.X, b.X);
        double maxX = Math.Max(a.X, b.X);
        double minY = Math.Min(a.Y, b.Y);
        double maxY = Math.Max(a.Y, b.Y);

        EarcutNode p = a;
        do
        {
            EarcutNode n = p.Next;
            if ((p.X > maxX && n.X > maxX) || (p.X < minX && n.X < minX) ||
                (p.Y > maxY && n.Y > maxY) || (p.Y < minY && n.Y < minY))
            {
                p = n;
                continue;
            }

            if (p.Index != a.Index && n.Index != a.Index && p.Index != b.Index && n.Index != b.Index &&
                Intersects(p, n, a, b))
            {
                return true;
            }

            p = n;
        }
        while (!ReferenceEquals(p, a));

        return false;
    }

    /// <summary>
    /// Determines whether a polygon diagonal is locally inside the polygon.
    /// </summary>
    /// <param name="a">The first node of the diagonal.</param>
    /// <param name="b">The second node of the diagonal.</param>
    /// <returns><see langword="true"/> if the diagonal is locally inside; otherwise <see langword="false"/>.</returns>
    private static bool LocallyInside(EarcutNode a, EarcutNode b)
        => Area(a.Prev, a, a.Next) < 0
        ? Area(a, b, a.Next) >= 0 && Area(a, a.Prev, b) >= 0
        : Area(a, b, a.Prev) < 0 || Area(a, a.Next, b) < 0;

    /// <summary>
    /// Determines whether the middle point of a polygon diagonal is inside the polygon.
    /// </summary>
    /// <param name="a">The first node of the diagonal.</param>
    /// <param name="b">The second node of the diagonal.</param>
    /// <returns><see langword="true"/> if the midpoint is inside; otherwise <see langword="false"/>.</returns>
    private static bool MiddleInside(EarcutNode a, EarcutNode b)
    {
        EarcutNode p = a;
        bool inside = false;
        double px = (a.X + b.X) / 2;
        double py = (a.Y + b.Y) / 2;
        do
        {
            EarcutNode n = p.Next;
            if (((p.Y > py) != (n.Y > py)) && (px < ((n.X - p.X) * (py - p.Y) / (n.Y - p.Y)) + p.X))
            {
                inside = !inside;
            }

            p = n;
        }
        while (!ReferenceEquals(p, a));

        return inside;
    }

    /// <summary>
    /// Links two polygon vertices with a bridge. If the vertices belong to the same ring, it splits
    /// the polygon into two; if one belongs to the outer ring and another to a hole, it merges it
    /// into a single ring.
    /// </summary>
    /// <param name="a">The first node.</param>
    /// <param name="b">The second node.</param>
    /// <returns>The node inserted before <paramref name="b"/>.</returns>
    private EarcutNode SplitPolygon(EarcutNode a, EarcutNode b)
    {
        EarcutNode a2 = this.CreateNode(a.Index, a.X, a.Y);
        EarcutNode b2 = this.CreateNode(b.Index, b.X, b.Y);
        EarcutNode an = a.Next;
        EarcutNode bp = b.Prev;

        a.Next = b;
        b.Prev = a;

        a2.Next = an;
        an.Prev = a2;

        b2.Next = a2;
        a2.Prev = b2;

        bp.Next = b2;
        b2.Prev = bp;

        return b2;
    }

    /// <summary>
    /// Creates a node and optionally links it with the previous one in a circular doubly linked
    /// list.
    /// </summary>
    /// <param name="i">The vertex index in the coordinates array.</param>
    /// <param name="x">The vertex x-coordinate.</param>
    /// <param name="y">The vertex y-coordinate.</param>
    /// <param name="last">The node to link to, or <see langword="null"/> to start a new ring.</param>
    /// <returns>The created node.</returns>
    private EarcutNode InsertNode(int i, double x, double y, EarcutNode? last)
    {
        EarcutNode p = this.CreateNode(i, x, y);

        if (last == null)
        {
            p.Prev = p;
            p.Next = p;
        }
        else
        {
            p.Next = last.Next;
            p.Prev = last;
            last.Next.Prev = p;
            last.Next = p;
        }

        return p;
    }

    /// <summary>
    /// Removes a node from the ring and from the z-order list.
    /// </summary>
    /// <param name="p">The node to remove.</param>
    private void RemoveNode(EarcutNode p)
    {
        p.Next.Prev = p.Prev;
        p.Prev.Next = p.Next;

        if (p.PrevZ != null)
        {
            p.PrevZ.NextZ = p.NextZ;
        }

        if (p.NextZ != null)
        {
            p.NextZ.PrevZ = p.PrevZ;
        }

        // Keep the hole-bridge index's block bboxes covering the healed prev->next edge.
        if (this.indexActive)
        {
            this.GrowBlock(p.Prev, p.Next);
        }
    }

    /// <summary>
    /// Rents a node from the pool, resetting all of its fields.
    /// </summary>
    /// <param name="i">The vertex index in the coordinates array.</param>
    /// <param name="x">The vertex x-coordinate.</param>
    /// <param name="y">The vertex y-coordinate.</param>
    /// <returns>The node.</returns>
    private EarcutNode CreateNode(int i, double x, double y)
    {
        EarcutNode node;
        if (this.nodeCount < this.nodePool.Count)
        {
            node = this.nodePool[this.nodeCount];
        }
        else
        {
            node = new EarcutNode();
            this.nodePool.Add(node);
        }

        this.nodeCount++;

        node.Index = i;
        node.X = x;
        node.Y = y;

        // Prev/Next are assigned by the caller before any read.
        node.Prev = null!;
        node.Next = null!;

        // z-order curve value; doubles as the owning block in the hole-bridge index during hole
        // elimination.
        node.Z = 0;
        node.PrevZ = null;
        node.NextZ = null;

        return node;
    }

    /// <summary>
    /// A hole together with the position it was queued at, used to keep the sort stable.
    /// </summary>
    private readonly struct HoleEntry
    {
        public HoleEntry(EarcutNode node, int order)
        {
            this.Node = node;
            this.Order = order;
        }

        /// <summary>
        /// Gets the leftmost node of the hole.
        /// </summary>
        public EarcutNode Node { get; }

        /// <summary>
        /// Gets the position the hole was queued at.
        /// </summary>
        public int Order { get; }
    }

    /// <summary>
    /// Orders holes by leftmost point and outgoing slope, falling back to the queue position so
    /// that the sort is stable.
    /// </summary>
    private sealed class HoleEntryComparer : IComparer<HoleEntry>
    {
        public static readonly HoleEntryComparer Instance = new();

        /// <inheritdoc/>
        public int Compare(HoleEntry x, HoleEntry y)
        {
            int result = CompareXYSlope(x.Node, y.Node);
            return result != 0 ? result : x.Order.CompareTo(y.Order);
        }
    }
}
