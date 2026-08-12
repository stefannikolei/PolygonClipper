# PolygonClipper

[![License: Six Labors Split](https://img.shields.io/badge/license-Six%20Labors%20Split-%23e30183)](https://github.com/SixLabors/PolygonClipper/blob/master/LICENSE)

## A Simple Algorithm for Boolean Operations on Polygons  

*Francisco Martínez, Carlos Ogayar, Juan R. Jiménez, Antonio J. Rueda*
  
https://sci-hub.se/10.1016/j.advengsoft.2013.04.004

This repository contains the beginnings of an attempted port of the original public domain C++ implementation by the main author of the paper Francisco Martínez.   

The original code can be found in the reference folder.  
  
The plan is to implement a performant port, add additional tests and some method by which to generate renders of output clipping operations.   
  
This is currently an intellectual exercise but I believe a C# port could be very useful in many applications if proven successful. 
  
All and any assistance is gratefully accepted. :heart:

## Triangulation

The library also contains a port of [earcut](https://github.com/mapbox/earcut) by Volodymyr Agafonkin, exposed as `PolygonTriangulator`. It turns a polygon into triangles, which is what a renderer needs to draw the result of a clipping operation.

```csharp
Polygon result = PolygonClipper.Union(subject, clip);

// Triangles as triplets of indices into the vertices of the polygon.
int[] triangles = PolygonTriangulator.Triangulate(result);

// Optionally refine the triangulation toward the constrained Delaunay triangulation.
FlatPolygon flat = PolygonTriangulator.Flatten(result);
PolygonTriangulator.Refine(triangles, flat.Vertices);
```

Every external contour is triangulated together with the hole contours that follow it, so results with several disjoint regions work as expected. Overloads accepting a `ReadOnlySpan<Vertex>` or a flat span of coordinates plus hole indices provide the original earcut contract, where the first ring is the external contour and the remaining rings are its holes.

The complete earcut test suite is ported alongside it, and the port produces the exact same triangulations as the JavaScript original.
