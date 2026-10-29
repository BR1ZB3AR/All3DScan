//! Reading files written by something other than All3DScan.
//!
//! A format reader that has only ever read its own output is barely tested:
//! real files come from Blender, MeshLab, CloudCompare and the scanner
//! vendors' own software, and that is where readers break. The fixtures in
//! tests/from_other_tools were written by Blender 5.2 — an icosphere of
//! radius 10 mm, 12 vertices and 20 triangles, with a colour per vertex.

use std::path::{Path, PathBuf};

use all3dscan_engine::interchange::{read, Scene};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/from_other_tools").join(name)
}

/// Every vertex's distance from the middle of the box it fills. For a
/// sphere these are all its radius, however the writer split the vertices.
fn radii(scene: &Scene) -> (f32, f32) {
    let (low, high) = scene.extent().expect("the fixture has no points");
    let centre = nalgebra::Point3::from((low.coords + high.coords) / 2.0);
    let mut least = f32::MAX;
    let mut most = f32::MIN;
    for point in &scene.points {
        let radius = (point - centre).norm();
        least = least.min(radius);
        most = most.max(radius);
    }
    (least, most)
}

#[test]
fn blenders_own_files_read_as_the_sphere_blender_made() {
    for name in [
        "blender_icosphere.ply",
        "blender_icosphere_ascii.ply",
        "blender_icosphere.stl",
        "blender_icosphere.obj",
    ] {
        let scene = read(&fixture(name)).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(scene.triangles(), 20, "{name} has the wrong triangle count");
        let (least, most) = radii(&scene);
        assert!(
            (least - 10.0).abs() < 1e-3 && (most - 10.0).abs() < 1e-3,
            "{name}: a sphere of radius 10 read as radius {least}-{most}"
        );
        // Blender splits vertices at its seams when it exports PLY, so the
        // count varies by format; every corner must still point somewhere.
        assert!(
            scene.indices.iter().all(|&i| (i as usize) < scene.points.len()),
            "{name} has a triangle pointing past its vertices"
        );
    }
}

#[test]
fn the_text_and_binary_forms_of_one_file_agree() {
    let binary = read(&fixture("blender_icosphere.ply")).unwrap();
    let text = read(&fixture("blender_icosphere_ascii.ply")).unwrap();
    assert_eq!(binary.points.len(), text.points.len());
    assert_eq!(binary.indices, text.indices);
    for (one, two) in binary.points.iter().zip(&text.points) {
        assert!((one - two).norm() < 1e-4, "{one:?} against {two:?}");
    }
}

#[test]
fn a_colour_per_vertex_written_by_another_tool_comes_through() {
    let scene = read(&fixture("blender_icosphere.ply")).unwrap();
    assert!(scene.has_colors(), "Blender wrote a colour per vertex and it was lost");
    assert!(
        scene.colors.iter().collect::<std::collections::HashSet<_>>().len() > 1,
        "every vertex came back the same colour"
    );
}

#[test]
fn normals_written_by_another_tool_come_through_and_point_outwards() {
    // OBJ is the fixture that carries them; on a sphere about the origin
    // every normal should point away from the middle.
    let scene = read(&fixture("blender_icosphere.obj")).unwrap();
    assert!(scene.has_normals(), "the OBJ's normals were lost");
    for (point, normal) in scene.points.iter().zip(&scene.normals) {
        assert!(
            normal.dot(&point.coords) > 0.0,
            "{normal:?} at {point:?} points into the sphere"
        );
    }
}
