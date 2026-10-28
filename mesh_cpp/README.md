# mesh_cpp — one surface from a scan's views (C++)

Wraps the Qt app's reconstruction (`../src/geometry/TsdfVolume`) so the Rust
engine can turn a scan into a closed surface: the depth views are melted into
a truncated signed distance field and the surface drawn through it with
Surface Nets, then eased with Taubin smoothing.

It has no build of its own: `engine_rust`'s build script compiles these files
along with the geometry they need, which depends on nothing but the standard
library. Points cross the boundary in millimetres, as the engine holds them;
the reconstruction underneath works in metres.

The surface comes back with a normal per vertex, worked out here from the
triangles sharing it, because the reconstruction does not carry them and the
viewport shades by them.
