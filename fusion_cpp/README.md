# fusion_cpp — building one cloud from a scan's frames (C++)

Wraps the Qt app's registration (`../src/processing/Registration`,
`../src/pointcloud`) so the Rust engine can fuse a scan: each frame is
aligned onto the model built so far with point-to-plane ICP, a coarse pass
then a fine one, seeded with the last frame's pose. A frame that does not sit
on the model well enough is reported as such and left out, rather than
smeared across it.

The settings are the ones the Qt app's pipeline uses
(`../src/processing/ScanPipeline.cpp`), kept in its units so the two can be
compared: a 1 mm model voxel, ICP on a cloud 1.8 times coarser, a 35 mm
coarse pass and an 8 mm fine one, and a frame accepted at 30% overlap within
4 mm and an RMS error of 2.5 mm or less.

It has no build of its own: `engine_rust`'s build script compiles these files
along with the registration they need, which depends on nothing but the
standard library and OpenMP. Points cross the boundary in millimetres, as the
engine holds them; the registration underneath works in metres.

`FrameFusion` keeps only what ICP needs to align against, and hands back the
pose it placed each frame at. The engine applies that pose to the frame's
normals as well, which the registration does not carry and a cylinder fit
needs, and keeps the model itself.
