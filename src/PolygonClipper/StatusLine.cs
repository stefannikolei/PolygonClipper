// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace PolygonClipper;

/// <summary>
/// Represents a status line for the sweep line algorithm, maintaining a sorted collection of sweep events.
/// <para>
/// Performance Characteristics:
/// - **Insertion**: O(n) in the worst case. The operation consists of:
///   1. A binary search (O(log n)) to determine the correct insertion point.
///   2. A shift operation to move subsequent elements in the list (O(k)), where k is the number of elements
///      after the insertion index. In the worst case, this can approach O(n).
/// - **Removal**: O(n) in the worst case. After finding the index of the element to remove, subsequent
///   elements in the list need to be shifted (O(k)), where k is the number of elements after the removed index.
/// - **Next/Previous Access**: O(1) after the index is known, as the list provides constant-time indexing.
/// </para>
/// The implementation ensures efficient neighbor traversal (next/previous) at O(1), making it suitable for
/// algorithms where neighboring elements are accessed frequently. The use of `BinarySearch` minimizes the cost
/// of insertion/removal compared to naive search-based approaches.
/// </summary>
[DebuggerDisplay("Count = {Count}")]
internal sealed class StatusLine
{
    private readonly List<SweepEvent> sortedEvents = [];
    private readonly SegmentComparer comparer = new();

    /// <summary>
    /// Gets the number of events in the status line.
    /// </summary>
    public int Count
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.sortedEvents.Count;
    }

    /// <summary>
    /// Gets the event at the specified index.
    /// </summary>
    /// <param name="index">The index of the event.</param>
    /// <returns>The sweep event at the given index.</returns>
    public SweepEvent this[int index]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.sortedEvents[index];
    }

    /// <summary>
    /// Inserts a sweep event into the status line, maintaining sorted order.
    /// </summary>
    /// <param name="e">The sweep event to insert.</param>
    /// <returns>The index where the event was inserted.</returns>
    public int Insert(SweepEvent e)
    {
        int index = this.BinarySearch(CollectionsMarshal.AsSpan(this.sortedEvents), e);
        if (index < 0)
        {
            index = ~index; // Get the correct insertion point
        }

        this.sortedEvents.Insert(index, e);
        this.Up(index);
        return index;
    }

    /// <summary>
    /// Removes a sweep event from the status line.
    /// </summary>
    /// <param name="index">The index of the event to remove.</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown if <paramref name="index"/> is less than 0 or greater than or equal to the number of events.
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RemoveAt(int index)
    {
        this.sortedEvents.RemoveAt(index);
        this.Down(index);
    }

    /// <summary>
    /// Gets the next sweep event relative to the given index.
    /// </summary>
    /// <param name="index">The reference index.</param>
    /// <returns>The next sweep event, or <c>null</c> if none exists.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public SweepEvent? Next(int index)
    {
        if (index >= 0 && index < this.sortedEvents.Count - 1)
        {
            return this.sortedEvents[index + 1];
        }

        return null;
    }

    /// <summary>
    /// Gets the previous sweep event relative to the given index.
    /// </summary>
    /// <param name="index">The reference index.</param>
    /// <returns>The previous sweep event, or <c>null</c> if none exists.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public SweepEvent? Prev(int index)
    {
        if (index > 0 && index < this.sortedEvents.Count)
        {
            return this.sortedEvents[index - 1];
        }

        return null;
    }

    /// <summary>
    /// Searches the sorted events for the given event.
    /// </summary>
    /// <param name="events">The sorted events to search.</param>
    /// <param name="value">The event to locate.</param>
    /// <returns>
    /// The index of the event if it is found; otherwise the bitwise complement of the index
    /// at which it would be inserted.
    /// </returns>
    /// <remarks>
    /// This mirrors the algorithm behind <see cref="List{T}.BinarySearch(T, IComparer{T})"/>
    /// exactly, including which element is returned when several compare equal. It is spelled
    /// out here only so the comparison is a direct call on the struct comparer rather than an
    /// interface call the JIT cannot inline - <see cref="SegmentComparer"/> is expensive enough
    /// that the call overhead is not the whole story, but it is measured O(log n) times per
    /// insertion.
    /// </remarks>
    private int BinarySearch(ReadOnlySpan<SweepEvent> events, SweepEvent value)
    {
        int lo = 0;
        int hi = events.Length - 1;

        while (lo <= hi)
        {
            int i = lo + ((hi - lo) >> 1);
            int order = this.comparer.Compare(events[i], value);

            if (order == 0)
            {
                return i;
            }

            if (order < 0)
            {
                lo = i + 1;
            }
            else
            {
                hi = i - 1;
            }
        }

        return ~lo;
    }

    private void Up(int index)
    {
        List<SweepEvent> e = this.sortedEvents;

        for (int i = index + 1; i < e.Count; i++)
        {
            e[i].PosSL = i;
        }
    }

    private void Down(int index)
    {
        List<SweepEvent> e = this.sortedEvents;

        for (int i = index; i < e.Count; i++)
        {
            e[i].PosSL = i;
        }
    }
}
