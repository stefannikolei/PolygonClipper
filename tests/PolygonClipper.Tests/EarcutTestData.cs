// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace PolygonClipper.Tests.TestCases;

/// <summary>
/// Provides access to the triangulation test fixtures ported from
/// <see href="https://github.com/mapbox/earcut"/>.
/// </summary>
internal static class EarcutTestData
{
    private static readonly Lazy<EarcutExpectations> ExpectationsLazy = new(LoadExpectations);

    /// <summary>
    /// Gets the expected triangle counts and deviations of each fixture.
    /// </summary>
    public static EarcutExpectations Expectations => ExpectationsLazy.Value;

    /// <summary>
    /// Gets the names of all fixtures with recorded expectations.
    /// </summary>
    /// <returns>The fixture names.</returns>
    public static IEnumerable<string> GetFixtureNames() => Expectations.Triangles.Keys;

    /// <summary>
    /// Reads the rings of the given fixture. The first ring is the external contour, the rest are
    /// holes.
    /// </summary>
    /// <param name="name">The name of the fixture, without extension.</param>
    /// <returns>The rings of the fixture.</returns>
    public static double[][][] GetRings(string name)
    {
        string path = Path.Combine(GetDirectory(), "Fixtures", name + ".json");
        return JsonSerializer.Deserialize<double[][][]>(File.ReadAllText(path))!;
    }

    /// <summary>
    /// Gets the full path of the packed vector tile fixture.
    /// </summary>
    /// <returns>The path of the fixture.</returns>
    public static string GetTilesFixturePath() => Path.Combine(GetDirectory(), "tiles-fixture.bin");

    private static string GetDirectory()
        => Path.Combine(TestEnvironment.GeoJsonTestDataFullPath, "Earcut");

    private static EarcutExpectations LoadExpectations()
    {
        string path = Path.Combine(GetDirectory(), "expected.json");
        return JsonSerializer.Deserialize<EarcutExpectations>(File.ReadAllText(path))!;
    }

    /// <summary>
    /// The contents of the fixture expectations file.
    /// </summary>
    internal sealed class EarcutExpectations
    {
        /// <summary>
        /// Gets the expected number of triangles of each fixture.
        /// </summary>
        [JsonPropertyName("triangles")]
        public Dictionary<string, int> Triangles { get; init; } = [];

        /// <summary>
        /// Gets the maximum allowed deviation of each fixture.
        /// </summary>
        [JsonPropertyName("errors")]
        public Dictionary<string, double> Errors { get; init; } = [];

        /// <summary>
        /// Gets the maximum allowed deviation of each fixture when it is rotated.
        /// </summary>
        [JsonPropertyName("errors-with-rotation")]
        public Dictionary<string, double> ErrorsWithRotation { get; init; } = [];

        /// <summary>
        /// Gets the maximum deviation allowed for the given fixture at the given rotation.
        /// </summary>
        /// <param name="name">The name of the fixture.</param>
        /// <param name="rotation">The rotation in degrees.</param>
        /// <returns>The maximum allowed deviation.</returns>
        public double GetExpectedDeviation(string name, int rotation)
        {
            if (rotation != 0 && this.ErrorsWithRotation.TryGetValue(name, out double rotated) && rotated != 0)
            {
                return rotated;
            }

            return this.Errors.TryGetValue(name, out double error) ? error : 0;
        }
    }
}
