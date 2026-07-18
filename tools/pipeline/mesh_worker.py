#!/usr/bin/env python3
"""JSON-line worker for hybrid C++/Python meshing.

The Qt app (C++) owns acquisition and UI. This process owns Open3D-based
fusion/meshing so algorithm work can stay in Python until the hot path is
ported. Protocol: one JSON object per stdin line, one JSON reply per stdout
line.

Commands:
  {"cmd":"ping"}
  {"cmd":"mesh","ply":"/path/in.ply","out":"/path/out.ply","method":"poisson","depth":9}
  {"cmd":"quit"}
"""
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any


def reply(payload: dict[str, Any]) -> None:
    sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def mesh_poisson(ply_path: Path, out_path: Path, depth: int) -> dict[str, Any]:
    try:
        import open3d as o3d
        import numpy as np
    except ImportError as exc:
        return {
            "ok": False,
            "error": f"Open3D is required for meshing ({exc}). "
            "Install with: pip install open3d",
        }

    cloud = o3d.io.read_point_cloud(str(ply_path))
    if cloud.is_empty():
        return {"ok": False, "error": f"empty point cloud: {ply_path}"}

    if not cloud.has_normals():
        cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
        )
        cloud.orient_normals_consistent_tangent_plane(30)

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        cloud, depth=depth
    )
    if densities:
        density = np.asarray(densities)
        mask = density < np.quantile(density, 0.02)
        mesh.remove_vertices_by_mask(mask)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_triangle_mesh(str(out_path), mesh):
        return {"ok": False, "error": f"failed to write mesh: {out_path}"}

    return {
        "ok": True,
        "mesh": str(out_path),
        "vertices": int(len(mesh.vertices)),
        "triangles": int(len(mesh.triangles)),
    }


def handle(message: dict[str, Any]) -> dict[str, Any]:
    cmd = message.get("cmd")
    if cmd == "ping":
        return {"ok": True, "pong": True, "worker": "mesh_worker"}
    if cmd == "quit":
        return {"ok": True, "quit": True}
    if cmd == "mesh":
        ply = Path(message["ply"])
        out = Path(message.get("out") or ply.with_suffix(".mesh.ply"))
        method = message.get("method", "poisson")
        depth = int(message.get("depth", 9))
        if method != "poisson":
            return {"ok": False, "error": f"unsupported method: {method}"}
        if not ply.is_file():
            return {"ok": False, "error": f"missing input PLY: {ply}"}
        return mesh_poisson(ply, out, depth)
    return {"ok": False, "error": f"unknown cmd: {cmd}"}


def main() -> int:
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            message = json.loads(line)
        except json.JSONDecodeError as exc:
            reply({"ok": False, "error": f"invalid json: {exc}"})
            continue
        response = handle(message)
        reply(response)
        if message.get("cmd") == "quit":
            return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
