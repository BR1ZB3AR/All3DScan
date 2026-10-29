# PoissonRecon, as vendored here

Screened Poisson surface reconstruction, by Michael Kazhdan and others.
All3DScan uses it to build a surface from a point cloud that has no depth
views behind it — one opened from a file, or one another scanner produced.
Scans taken here are meshed from their own depth views instead
(`src/geometry/TsdfVolume`), which is the better answer when those views
exist.

## Where it came from

- Upstream: <https://github.com/mkazhdan/PoissonRecon>
- Taken at commit `262b0f5`, "Version 8.76", fetched 2026-10-29.
- 59 of the 95 files in upstream `Src/` are here: the set the compiler
  actually pulls in for reconstruction, found with `g++ -MM`. The rest are
  the command-line programs, the client/server code, image handling, and the
  bundled copies of JPEG, PNG and ZLIB — none of which this uses. Nothing
  has been edited; the wrapper around them is in `mesh_cpp/`.

## Licence

**Read this before shipping a binary.** Upstream carries two different
notices and they do not say the same thing:

- the repository's own `LICENSE` file is **MIT**, "Copyright (c) 2015
  mkazhdan" — kept here as `LICENSE`;
- but **94 of the 95 files in `Src/` carry BSD-3-Clause headers**, and every
  file vendored here is one of them.

Both are permissive and both are compatible with All3DScan's own MIT
licence. BSD-3-Clause asks for more than MIT does, so that is what this
directory is treated as carrying:

1. the copyright notice, this list of conditions and the disclaimer must be
   kept with the source (they are, at the top of every file);
2. a binary distribution must reproduce them in its documentation or other
   materials;
3. the names of Johns Hopkins University, the Georgia Institute of
   Technology, or the contributors may not be used to endorse or promote
   All3DScan without their written permission.

The copyright is held by several parties across these files, including
Michael Kazhdan and Matthew Bolitho (2006-2023) and the Georgia Institute of
Technology (`Ply.h`, `PlyFile.h`, 2020).

## What has and has not been checked

Checked: that every file came from the commit above unedited; the licence
notice each one carries; that the set compiles with GCC 16.2.1 at C++17 with
OpenMP and links against nothing but libgomp; and that it reconstructs
correctly — a 25 mm sphere of oriented points comes back spanning 50.01 mm,
and a real 99,444-point POP 4 scan meshes in about 1.2 s.

Not checked: the numerical internals, line by line. This is 1.8 MB of
template-heavy solver code and nobody here has read all of it. It is widely
used and comes from its authors' own repository, which is the basis for
trusting it; that is not the same as having audited it.
