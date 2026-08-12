# Third Party Notices

This repository contains code derived from the following third party projects.

## earcut

`src/PolygonClipper/PolygonTriangulator.cs`, `src/PolygonClipper/EarcutEngine.cs`,
`src/PolygonClipper/EarcutNode.cs`, `src/PolygonClipper/DelaunayRefiner.cs` and
`src/PolygonClipper/FlatPolygon.cs` are a port of [earcut](https://github.com/mapbox/earcut) by
Volodymyr Agafonkin. The test fixtures under `tests/TestData/Earcut` and the tests deriving from
them are taken from the same project.

```
ISC License

Copyright (c) 2026, Mapbox

Permission to use, copy, modify, and/or distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright notice
and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF
THIS SOFTWARE.
```

## Martinez Polygon Clipping

Portions of the unit tests are derived from the
[Martinez Polygon Clipping Library](https://github.com/w8r/martinez) by Dmitriy Zaitsev and
contributors, used under the terms of the MIT License.
