// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Diagnostics;

namespace PolygonClipper;

/// <summary>
/// A vertex in a circular doubly linked list representing a polygon ring.
/// <see cref="Prev"/>/<see cref="Next"/> are always linked (set immediately after the node is
/// created), so they are never <see langword="null"/> once the node is part of a ring;
/// <see cref="PrevZ"/>/<see cref="NextZ"/> are the z-order list links and are
/// <see langword="null"/> at the ends.
/// </summary>
[DebuggerDisplay("I = {Index}, X = {X}, Y = {Y}")]
internal sealed class EarcutNode
{
    /// <summary>
    /// Gets or sets the vertex index in the coordinates array.
    /// </summary>
    public int Index { get; set; }

    /// <summary>
    /// Gets or sets the vertex x-coordinate.
    /// </summary>
    public double X { get; set; }

    /// <summary>
    /// Gets or sets the vertex y-coordinate.
    /// </summary>
    public double Y { get; set; }

    /// <summary>
    /// Gets or sets the previous vertex node in the polygon ring.
    /// </summary>
    public EarcutNode Prev { get; set; } = null!;

    /// <summary>
    /// Gets or sets the next vertex node in the polygon ring.
    /// </summary>
    public EarcutNode Next { get; set; } = null!;

    /// <summary>
    /// Gets or sets the z-order curve value. Doubles as the owning block index while holes
    /// are being eliminated.
    /// </summary>
    public int Z { get; set; }

    /// <summary>
    /// Gets or sets the previous node in z-order.
    /// </summary>
    public EarcutNode? PrevZ { get; set; }

    /// <summary>
    /// Gets or sets the next node in z-order.
    /// </summary>
    public EarcutNode? NextZ { get; set; }
}
