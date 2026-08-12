// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections.Generic;
using System.IO;

namespace PolygonClipper.Tests;

/// <summary>
/// Reads the packed vector tile fixture used by the triangulation tests, ported from
/// <see href="https://github.com/mapbox/earcut"/>.
/// </summary>
/// <remarks>
/// <para>
/// The fixture contains length-delimited packed-varint vector tile geometry blobs, one per polygon
/// feature. They are decoded with a small varint reader, reconstructing polygons by splitting
/// multipolygons and classifying holes by signed area, per the vector tile specification.
/// </para>
/// <para>
/// The file format (little-endian LEB128 unsigned varints throughout) is:
/// </para>
/// <code>
///   file    := tile*                     (repeated until EOF)
///   tile    := zoom featureCount feature*
///   feature := geomLen geom              (geomLen = number of varints in geom)
///   geom    := uint32*                   (raw command/parameter integers)
/// </code>
/// <para>
/// The geometry integers are the native vector tile polygon encoding: MoveTo/LineTo/ClosePath
/// command integers interleaved with zigzag delta-encoded coordinate pairs.
/// </para>
/// </remarks>
internal static class MvtTilesFixture
{
    /// <summary>
    /// Reads every polygon of the fixture at the given path.
    /// </summary>
    /// <param name="path">The path of the fixture.</param>
    /// <returns>The polygons in the flat form the triangulator accepts.</returns>
    public static List<MvtPolygon> Read(string path)
    {
        byte[] buf = File.ReadAllBytes(path);
        int pos = 0;
        List<MvtPolygon> polys = [];

        while (pos < buf.Length)
        {
            int z = (int)ReadVarint(buf, ref pos);
            uint features = ReadVarint(buf, ref pos);

            for (uint feature = 0; feature < features; feature++)
            {
                uint count = ReadVarint(buf, ref pos);

                uint[] geom = new uint[count];
                for (int i = 0; i < count; i++)
                {
                    geom[i] = ReadVarint(buf, ref pos);
                }

                List<List<double[]>>? current = null;

                foreach (List<double[]> ring in DecodeRings(geom))
                {
                    if (ring.Count < 3)
                    {
                        continue;
                    }

                    double area = RingArea(ring);
                    if (area == 0)
                    {
                        continue;
                    }

                    if (area > 0)
                    {
                        if (current != null)
                        {
                            polys.Add(new MvtPolygon(Flatten(current), z));
                        }

                        current = [ring];
                    }
                    else
                    {
                        current?.Add(ring);
                    }
                }

                if (current != null)
                {
                    polys.Add(new MvtPolygon(Flatten(current), z));
                }
            }
        }

        return polys;
    }

    private static FlatPolygon Flatten(List<List<double[]>> rings)
    {
        int count = 0;
        for (int i = 0; i < rings.Count; i++)
        {
            count += rings[i].Count;
        }

        double[] vertices = new double[count * 2];
        int[] holes = new int[rings.Count - 1];
        int index = 0;
        int holeIndex = 0;
        int prevLen = 0;

        for (int i = 0; i < rings.Count; i++)
        {
            List<double[]> ring = rings[i];
            foreach (double[] p in ring)
            {
                vertices[index++] = p[0];
                vertices[index++] = p[1];
            }

            if (prevLen != 0)
            {
                holeIndex += prevLen;
                holes[i - 1] = holeIndex;
            }

            prevLen = ring.Count;
        }

        return new FlatPolygon(vertices, holes, 2);
    }

    private static uint ReadVarint(byte[] buf, ref int pos)
    {
        uint val = 0;
        int shift = 0;
        byte b;
        do
        {
            b = buf[pos++];
            val |= (uint)(b & 0x7f) << shift;
            shift += 7;
        }
        while ((b & 0x80) != 0);

        return val;
    }

    private static List<List<double[]>> DecodeRings(uint[] geom)
    {
        List<List<double[]>> rings = [];
        int x = 0;
        int y = 0;
        List<double[]>? ring = null;
        int i = 0;

        while (i < geom.Length)
        {
            uint cmd = geom[i] & 0x7;
            uint count = geom[i] >> 3;
            i++;

            if (cmd == 1)
            {
                for (uint k = 0; k < count; k++)
                {
                    x += ZigZagDecode(geom[i++]);
                    y += ZigZagDecode(geom[i++]);
                    if (ring != null)
                    {
                        rings.Add(ring);
                    }

                    ring = [[x, y]];
                }
            }
            else if (cmd == 2)
            {
                for (uint k = 0; k < count; k++)
                {
                    x += ZigZagDecode(geom[i++]);
                    y += ZigZagDecode(geom[i++]);
                    ring!.Add([x, y]);
                }
            }
            else if (cmd == 7 && ring != null)
            {
                rings.Add(ring);
                ring = null;
            }
        }

        if (ring != null)
        {
            rings.Add(ring);
        }

        return rings;
    }

    private static int ZigZagDecode(uint n)
    {
        int v = (int)n;
        return (v >> 1) ^ -(v & 1);
    }

    private static double RingArea(List<double[]> ring)
    {
        double sum = 0;
        for (int i = 0, j = ring.Count - 1; i < ring.Count; j = i++)
        {
            sum += (ring[j][0] - ring[i][0]) * (ring[i][1] + ring[j][1]);
        }

        return sum / 2;
    }
}

/// <summary>
/// A polygon read from the packed vector tile fixture.
/// </summary>
internal readonly struct MvtPolygon
{
    public MvtPolygon(FlatPolygon data, int zoom)
    {
        this.Data = data;
        this.Zoom = zoom;
    }

    /// <summary>
    /// Gets the polygon in the flat form the triangulator accepts.
    /// </summary>
    public FlatPolygon Data { get; }

    /// <summary>
    /// Gets the zoom level of the tile the polygon was read from.
    /// </summary>
    public int Zoom { get; }
}
