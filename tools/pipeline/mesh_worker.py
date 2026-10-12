#!/usr/bin/env python3
"""JSON-line worker for hybrid C++/Python meshing / registration.

The Qt app (C++) owns acquisition and UI. This process owns Open3D-based
fusion/meshing so algorithm work can stay in Python until the hot path is
ported. Protocol: one JSON object per stdin line, one JSON reply per stdout
line.

Commands:
  {"cmd":"ping"}
  {"cmd":"capabilities"}
  {"cmd":"mesh","ply":"/path/in.ply","out":"/path/out.ply","method":"poisson","depth":9}
  {"cmd":"register_frame","source":"/path/src.ply","target":"/path/tgt.ply","out":"/path/aligned.ply"}
  {"cmd":"smooth_mesh","mesh":"/path/in.ply","out":"/path/out.ply","iterations":10}
  {"cmd":"tsdf","manifest":"/path/manifest.json"}
  {"cmd":"quit"}

TSDF manifest schema (see tsdf_reconstruct): volumetric depth fusion of the
per-keyframe metric depth maps + poses captured during live scanning. This is
closer to RevoScan's live model than per-frame ICP + a final Poisson pass.
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
    # ASCII PLY so the lightweight C++ parser in ScanPipeline can read it.
    if not o3d.io.write_triangle_mesh(str(out_path), mesh, write_ascii=True):
        return {"ok": False, "error": f"failed to write mesh: {out_path}"}

    return {
        "ok": True,
        "mesh": str(out_path),
        "vertices": int(len(mesh.vertices)),
        "triangles": int(len(mesh.triangles)),
    }


def tsdf_reconstruct(manifest_path: Path) -> dict[str, Any]:
    """Volumetric TSDF fusion of per-keyframe metric depth maps + poses.

    Manifest JSON:
      {
        "out": "/path/out.ply",
        "voxel_length": 0.003,      # metres
        "sdf_trunc": 0.012,         # metres
        "depth_scale": 1000.0,      # depth units per metre (mm → 1000)
        "depth_min": 0.15,          # metres, crop band
        "depth_max": 0.60,          # metres
        "frames": [
          {"depth": "/path/kf000.bin", "width": 320, "height": 512,
           "fx": .., "fy": .., "cx": .., "cy": ..,
           "pose": [16 floats, row-major camera->world]}
        ]
      }
    Depth bins are raw little-endian uint16, width*height, row-major.
    """
    try:
        import open3d as o3d
        import numpy as np
    except ImportError as exc:
        return {"ok": False, "error": f"Open3D required for TSDF ({exc})"}

    try:
        manifest = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        return {"ok": False, "error": f"bad manifest: {exc}"}

    frames = manifest.get("frames", [])
    if not frames:
        return {"ok": False, "error": "no keyframes in manifest"}

    out_path = Path(manifest["out"])
    voxel_length = float(manifest.get("voxel_length", 0.003))
    sdf_trunc = float(manifest.get("sdf_trunc", max(voxel_length * 4.0, 0.01)))
    depth_scale = float(manifest.get("depth_scale", 1000.0))
    depth_min = float(manifest.get("depth_min", 0.10))
    depth_max = float(manifest.get("depth_max", 1.0))

    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_length,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    integrated = 0
    for frame in frames:
        width = int(frame["width"])
        height = int(frame["height"])
        depth = np.fromfile(frame["depth"], dtype="<u2")
        if depth.size != width * height:
            continue
        depth = depth.reshape(height, width).astype(np.uint16)

        lo = int(round(depth_min * depth_scale))
        hi = int(round(depth_max * depth_scale))
        mask = (depth < lo) | (depth > hi)
        if mask.any():
            depth = depth.copy()
            depth[mask] = 0
        if not np.any(depth):
            continue

        depth_img = o3d.geometry.Image(np.ascontiguousarray(depth))
        color_path = frame.get("color")
        color = None
        if color_path:
            raw = np.fromfile(color_path, dtype=np.uint8)
            if raw.size == width * height * 3:
                color = np.ascontiguousarray(raw.reshape(height, width, 3))
        if color is None:
            color = np.full((height, width, 3), 160, dtype=np.uint8)
        color_img = o3d.geometry.Image(color)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_img,
            depth_img,
            depth_scale=depth_scale,
            depth_trunc=depth_max,
            convert_rgb_to_intensity=False,
        )
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width, height,
            float(frame["fx"]), float(frame["fy"]),
            float(frame["cx"]), float(frame["cy"]),
        )
        pose = np.asarray(frame["pose"], dtype=np.float64).reshape(4, 4)
        try:
            extrinsic = np.linalg.inv(pose)  # world -> camera
        except np.linalg.LinAlgError:
            continue
        volume.integrate(rgbd, intrinsic, extrinsic)
        integrated += 1

    if integrated == 0:
        return {"ok": False, "error": "no keyframes integrated (empty/invalid depth)"}

    mesh = volume.extract_triangle_mesh()
    if mesh.is_empty():
        return {"ok": False, "error": "TSDF produced an empty mesh"}
    mesh.compute_vertex_normals()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_triangle_mesh(str(out_path), mesh, write_ascii=True):
        return {"ok": False, "error": f"failed to write mesh: {out_path}"}

    return {
        "ok": True,
        "mesh": str(out_path),
        "vertices": int(len(mesh.vertices)),
        "triangles": int(len(mesh.triangles)),
        "integrated": integrated,
    }


def register_frame(source: Path, target: Path, out: Path) -> dict[str, Any]:
    try:
        import open3d as o3d
        import numpy as np
    except ImportError as exc:
        return {"ok": False, "error": f"Open3D required ({exc})"}

    src = o3d.io.read_point_cloud(str(source))
    tgt = o3d.io.read_point_cloud(str(target))
    if src.is_empty() or tgt.is_empty():
        return {"ok": False, "error": "empty source/target cloud"}

    src = src.voxel_down_sample(0.003)
    tgt = tgt.voxel_down_sample(0.003)
    src.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
    )
    tgt.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
    )

    result = o3d.pipelines.registration.registration_icp(
        src,
        tgt,
        0.02,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    src.transform(result.transformation)
    out.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(out), src)
    return {
        "ok": True,
        "fitness": float(result.fitness),
        "rmse": float(result.inlier_rmse),
        "out": str(out),
        "transform": result.transformation.reshape(-1).tolist(),
    }


def smooth_mesh(mesh_path: Path, out_path: Path, iterations: int) -> dict[str, Any]:
    try:
        import open3d as o3d
    except ImportError as exc:
        return {"ok": False, "error": f"Open3D required ({exc})"}
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    if mesh.is_empty():
        return {"ok": False, "error": "empty mesh"}
    mesh = mesh.filter_smooth_taubin(number_of_iterations=iterations)
    mesh.compute_vertex_normals()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_triangle_mesh(str(out_path), mesh, write_ascii=True):
        return {"ok": False, "error": "failed to write smoothed mesh"}
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
    if cmd == "capabilities":
        return {
            "ok": True,
            "commands": [
                "ping",
                "capabilities",
                "mesh",
                "tsdf",
                "register_frame",
                "smooth_mesh",
                "quit",
            ],
        }
    if cmd == "quit":
        return {"ok": True, "quit": True}
    if cmd == "tsdf":
        manifest = message.get("manifest")
        if not manifest:
            return {"ok": False, "error": "tsdf requires a manifest path"}
        manifest_path = Path(manifest)
        if not manifest_path.is_file():
            return {"ok": False, "error": f"missing manifest: {manifest_path}"}
        return tsdf_reconstruct(manifest_path)
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
    if cmd == "register_frame":
        return register_frame(
            Path(message["source"]), Path(message["target"]), Path(message["out"])
        )
    if cmd == "smooth_mesh":
        return smooth_mesh(
            Path(message["mesh"]),
            Path(message["out"]),
            int(message.get("iterations", 10)),
        )
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
