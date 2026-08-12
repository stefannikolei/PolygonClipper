// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections;
using System.Collections.Generic;

namespace PolygonClipper;

/// <summary>
/// Compares two <see cref="SweepEvent"/> instances for sorting in the event queue.
/// </summary>
/// <remarks>
/// This is a struct so that generic callers constrained to <c>struct, IComparer&lt;T&gt;</c> get
/// their own JIT instantiation, in which <see cref="Compare(SweepEvent, SweepEvent)"/> is a
/// direct call that can be inlined. As a class it shared the <c>__Canon</c> instantiation and
/// every comparison went through an interface call.
/// </remarks>
internal readonly struct SweepEventComparer : IComparer<SweepEvent>, IComparer
{
    /// <inheritdoc/>
    public int Compare(SweepEvent? x, SweepEvent? y)
    {
        if (x == null)
        {
            return -1;
        }

        if (y == null)
        {
            return 1;
        }

        // Compare by x-coordinate
        if (x.Point.X > y.Point.X)
        {
            return 1;
        }

        if (x.Point.X < y.Point.X)
        {
            return -1;
        }

        // Compare by y-coordinate when x-coordinates are the same
        if (x.Point.Y != y.Point.Y)
        {
            return x.Point.Y > y.Point.Y ? 1 : -1;
        }

        // Compare left vs. right endpoint
        if (x.Left != y.Left)
        {
            return x.Left ? 1 : -1;
        }

        // Compare collinearity using signed area
        double area = PolygonUtilities.SignedArea(x.Point, x.OtherPoint, y.OtherPoint);
        if (area != 0)
        {
            return x.Below(y.OtherPoint) ? -1 : 1;
        }

        // Compare by polygon type: subject polygons have higher priority
        return x.PolygonType != PolygonType.Subject && y.PolygonType == PolygonType.Subject ? 1 : -1;
    }

    /// <inheritdoc/>
    public int Compare(object? x, object? y)
    {
        if (x is SweepEvent a && y is SweepEvent b)
        {
            return this.Compare(a, b);
        }

        throw new ArgumentException("Both arguments must be of type SweepEvent.", nameof(x));
    }
}
