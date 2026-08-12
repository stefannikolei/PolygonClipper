// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace PolygonClipper;

/// <summary>
/// Represents a status line for the sweep line algorithm, maintaining a sorted collection of
/// sweep events.
/// <para>
/// Implemented as an AVL tree, so insertion, removal and neighbour lookup are all O(log n), and
/// - crucially - the handle an event holds stays valid when other events are inserted or removed
/// around it.
/// </para>
/// <para>
/// The previous implementation kept a sorted <see cref="System.Collections.Generic.List{T}"/> and
/// identified events by their index. Because inserting into or removing from a list shifts every
/// later element, each operation had to rewrite <c>PosSL</c> on all following events: an O(n)
/// chain of random writes into separate heap objects, on top of the list's own O(n) shift. On a
/// stack of long overlapping segments that renumbering dominated everything else - fifty thousand
/// vertices produced over a billion of those writes.
/// </para>
/// <para>
/// The original C++ implementation does not have this problem: it stores the segments in a
/// <c>std::set</c> and keeps the set iterator in <c>posSL</c>, and set iterators survive
/// unrelated insertions and erasures. The node handles here serve the same purpose.
/// </para>
/// <para>
/// Nodes live in parallel arrays rather than as objects, and freed slots are recycled, so the
/// tree costs no per-insertion allocation and its links stay contiguous in memory. That also
/// keeps <see cref="SweepEvent.PosSL"/> a plain <see cref="int"/>, leaving the size of a sweep
/// event unchanged.
/// </para>
/// </summary>
[DebuggerDisplay("Count = {Count}")]
internal sealed class StatusLine
{
    /// <summary>
    /// Sentinel for "no node", used in place of a null reference.
    /// </summary>
    private const int Nil = -1;

    private readonly SegmentComparer comparer = new();

    private SweepEvent[] events;
    private int[] left;
    private int[] right;
    private int[] parent;

    /// <summary>
    /// Node heights. An AVL tree of n nodes is at most ~1.44 * log2(n) deep, so a byte is
    /// ample and keeps the array dense.
    /// </summary>
    private byte[] height;

    private int root = Nil;

    /// <summary>
    /// The number of slots handed out so far; slots beyond this have never been used.
    /// </summary>
    private int allocated;

    /// <summary>
    /// Head of the free slot list. Freed slots link to the next through <see cref="parent"/>.
    /// </summary>
    private int free = Nil;

    /// <summary>
    /// Initializes a new instance of the <see cref="StatusLine"/> class.
    /// </summary>
    public StatusLine()
    {
        const int capacity = 16;
        this.events = new SweepEvent[capacity];
        this.left = new int[capacity];
        this.right = new int[capacity];
        this.parent = new int[capacity];
        this.height = new byte[capacity];
    }

    /// <summary>
    /// Gets the number of events in the status line.
    /// </summary>
    public int Count { get; private set; }

    /// <summary>
    /// Inserts a sweep event into the status line, maintaining sorted order.
    /// </summary>
    /// <param name="e">The sweep event to insert.</param>
    /// <returns>
    /// A handle identifying the event's node. It remains valid until that event is removed,
    /// regardless of what else is inserted or removed in the meantime.
    /// </returns>
    public int Insert(SweepEvent e)
    {
        int node = this.Allocate(e);
        this.Count++;

        if (this.root == Nil)
        {
            this.root = node;
            return node;
        }

        // SegmentComparer only returns zero for reference-equal events, and an event is never
        // inserted twice, so there is no equal-key case to resolve here.
        int current = this.root;
        while (true)
        {
            if (this.comparer.Compare(e, this.events[current]) < 0)
            {
                if (this.left[current] == Nil)
                {
                    this.left[current] = node;
                    break;
                }

                current = this.left[current];
            }
            else
            {
                if (this.right[current] == Nil)
                {
                    this.right[current] = node;
                    break;
                }

                current = this.right[current];
            }
        }

        this.parent[node] = current;
        this.Rebalance(current);
        return node;
    }

    /// <summary>
    /// Removes the event identified by the given handle.
    /// </summary>
    /// <param name="node">The handle returned by <see cref="Insert"/>.</param>
    public void RemoveAt(int node)
    {
        int rebalanceFrom;

        if (this.left[node] == Nil || this.right[node] == Nil)
        {
            // At most one child: lift it into this node's place.
            int child = this.left[node] != Nil ? this.left[node] : this.right[node];
            rebalanceFrom = this.parent[node];
            this.Transplant(node, child);
        }
        else
        {
            // Two children. The usual trick of copying the successor's payload into this node
            // cannot be used here: callers hold node handles, and moving an event to a different
            // node would invalidate the handle that other event is still holding. So the
            // successor node itself is spliced into this node's position instead.
            int successor = Minimum(this.right[node]);

            if (this.parent[successor] == node)
            {
                rebalanceFrom = successor;
                this.Transplant(node, successor);
                this.left[successor] = this.left[node];
                this.parent[this.left[successor]] = successor;
            }
            else
            {
                rebalanceFrom = this.parent[successor];

                // The successor is a leftmost descendant, so it has no left child.
                this.Transplant(successor, this.right[successor]);
                this.Transplant(node, successor);

                this.left[successor] = this.left[node];
                this.parent[this.left[successor]] = successor;
                this.right[successor] = this.right[node];
                this.parent[this.right[successor]] = successor;
            }

            this.height[successor] = this.height[node];
        }

        this.Count--;
        this.Release(node);

        if (rebalanceFrom != Nil)
        {
            this.Rebalance(rebalanceFrom);
        }
    }

    /// <summary>
    /// Gets the event that follows the given one in sorted order.
    /// </summary>
    /// <param name="node">The handle of the reference event.</param>
    /// <returns>The next sweep event, or <see langword="null"/> if none exists.</returns>
    public SweepEvent? Next(int node)
    {
        if (this.right[node] != Nil)
        {
            return this.events[Minimum(this.right[node])];
        }

        // Walk up until we come from a left child; that ancestor is the successor.
        int current = node;
        int up = this.parent[current];
        while (up != Nil && this.right[up] == current)
        {
            current = up;
            up = this.parent[up];
        }

        return up == Nil ? null : this.events[up];
    }

    /// <summary>
    /// Gets the event that precedes the given one in sorted order.
    /// </summary>
    /// <param name="node">The handle of the reference event.</param>
    /// <returns>The previous sweep event, or <see langword="null"/> if none exists.</returns>
    public SweepEvent? Prev(int node)
    {
        if (this.left[node] != Nil)
        {
            return this.events[Maximum(this.left[node])];
        }

        // Walk up until we come from a right child; that ancestor is the predecessor.
        int current = node;
        int up = this.parent[current];
        while (up != Nil && this.left[up] == current)
        {
            current = up;
            up = this.parent[up];
        }

        return up == Nil ? null : this.events[up];
    }

    /// <summary>
    /// Walks up from the given node to the root, restoring heights and AVL balance.
    /// </summary>
    private void Rebalance(int node)
    {
        while (node != Nil)
        {
            this.UpdateHeight(node);
            int balance = this.BalanceOf(node);

            if (balance > 1)
            {
                if (this.BalanceOf(this.left[node]) < 0)
                {
                    this.RotateLeft(this.left[node]);
                }

                node = this.RotateRight(node);
            }
            else if (balance < -1)
            {
                if (this.BalanceOf(this.right[node]) > 0)
                {
                    this.RotateRight(this.right[node]);
                }

                node = this.RotateLeft(node);
            }

            node = this.parent[node];
        }
    }

    /// <summary>
    /// Replaces the subtree rooted at <paramref name="target"/> with the one rooted at
    /// <paramref name="replacement"/>, which may be <see cref="Nil"/>.
    /// </summary>
    private void Transplant(int target, int replacement)
    {
        int up = this.parent[target];
        if (up == Nil)
        {
            this.root = replacement;
        }
        else if (this.left[up] == target)
        {
            this.left[up] = replacement;
        }
        else
        {
            this.right[up] = replacement;
        }

        if (replacement != Nil)
        {
            this.parent[replacement] = up;
        }
    }

    /// <summary>
    /// Rotates the subtree rooted at <paramref name="node"/> to the right.
    /// </summary>
    /// <returns>The new root of that subtree.</returns>
    private int RotateRight(int node)
    {
        int pivot = this.left[node];

        this.left[node] = this.right[pivot];
        if (this.right[pivot] != Nil)
        {
            this.parent[this.right[pivot]] = node;
        }

        this.Transplant(node, pivot);

        this.right[pivot] = node;
        this.parent[node] = pivot;

        this.UpdateHeight(node);
        this.UpdateHeight(pivot);
        return pivot;
    }

    /// <summary>
    /// Rotates the subtree rooted at <paramref name="node"/> to the left.
    /// </summary>
    /// <returns>The new root of that subtree.</returns>
    private int RotateLeft(int node)
    {
        int pivot = this.right[node];

        this.right[node] = this.left[pivot];
        if (this.left[pivot] != Nil)
        {
            this.parent[this.left[pivot]] = node;
        }

        this.Transplant(node, pivot);

        this.left[pivot] = node;
        this.parent[node] = pivot;

        this.UpdateHeight(node);
        this.UpdateHeight(pivot);
        return pivot;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int HeightOf(int node) => node == Nil ? 0 : this.height[node];

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void UpdateHeight(int node)
        => this.height[node] = (byte)(1 + Math.Max(this.HeightOf(this.left[node]), this.HeightOf(this.right[node])));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int BalanceOf(int node) => this.HeightOf(this.left[node]) - this.HeightOf(this.right[node]);

    private int Minimum(int node)
    {
        while (this.left[node] != Nil)
        {
            node = this.left[node];
        }

        return node;
    }

    private int Maximum(int node)
    {
        while (this.right[node] != Nil)
        {
            node = this.right[node];
        }

        return node;
    }

    private int Allocate(SweepEvent e)
    {
        int node;
        if (this.free != Nil)
        {
            node = this.free;
            this.free = this.parent[node];
        }
        else
        {
            if (this.allocated == this.events.Length)
            {
                this.Grow();
            }

            node = this.allocated++;
        }

        this.events[node] = e;
        this.left[node] = Nil;
        this.right[node] = Nil;
        this.parent[node] = Nil;
        this.height[node] = 1;
        return node;
    }

    private void Release(int node)
    {
        // Drop the reference so a removed event is not kept alive by the status line.
        this.events[node] = null!;
        this.parent[node] = this.free;
        this.free = node;
    }

    private void Grow()
    {
        int capacity = this.events.Length * 2;
        Array.Resize(ref this.events, capacity);
        Array.Resize(ref this.left, capacity);
        Array.Resize(ref this.right, capacity);
        Array.Resize(ref this.parent, capacity);
        Array.Resize(ref this.height, capacity);
    }
}
