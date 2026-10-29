//! Reading and writing the files 3D scanners exchange.
//!
//! Every scanner writes a point cloud or a mesh in one of a handful of
//! formats, whatever its own protocol looks like, so this is where a scanner
//! All3DScan cannot drive still becomes a scanner All3DScan can work with:
//! bring in what it exported and the cleaning, fusing, meshing and fitting
//! all apply.
//!
//! Lengths are taken as the file gives them and never rescaled here. The
//! engine works in millimetres and most scanners write millimetres, but some
//! write metres; `Scene::extent` is there so a caller can notice and ask.

use std::collections::HashMap;
use std::fmt;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

use nalgebra::{Point3, Vector3};

/// What a scan file holds.
///
/// Points are always there. The rest is whatever the file carried, so an
/// empty `colors` means the file had no colour, not that every point is
/// black; `indices` is empty for a plain cloud.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Scene {
    pub points: Vec<Point3<f32>>,
    pub colors: Vec<[u8; 3]>,
    pub normals: Vec<Vector3<f32>>,
    /// Three indices into `points` per triangle.
    pub indices: Vec<u32>,
}

impl Scene {
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn triangles(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn has_colors(&self) -> bool {
        self.colors.len() == self.points.len() && !self.points.is_empty()
    }

    pub fn has_normals(&self) -> bool {
        self.normals.len() == self.points.len() && !self.points.is_empty()
    }

    /// The box the points fill: (lowest corner, highest corner). None when
    /// there are no points. A caller showing this to someone can tell
    /// millimetres from metres by how big it is.
    pub fn extent(&self) -> Option<(Point3<f32>, Point3<f32>)> {
        let mut points = self.points.iter().filter(|p| p.coords.iter().all(|c| c.is_finite()));
        let first = points.next()?;
        let (mut low, mut high) = (*first, *first);
        for point in points {
            for axis in 0..3 {
                low[axis] = low[axis].min(point[axis]);
                high[axis] = high[axis].max(point[axis]);
            }
        }
        Some((low, high))
    }

    /// Multiplies every length by `by`, for a file written in other units.
    pub fn scale(&mut self, by: f32) {
        for point in &mut self.points {
            *point *= by;
        }
    }

    /// Drops colours and normals that do not have one entry per point, which
    /// a file is free to leave half-filled.
    fn tidy(&mut self) {
        if self.colors.len() != self.points.len() {
            self.colors.clear();
        }
        if self.normals.len() != self.points.len() {
            self.normals.clear();
        }
        let points = self.points.len() as u32;
        if self.indices.iter().any(|&i| i >= points) || self.indices.len() % 3 != 0 {
            self.indices.clear();
        }
    }
}

/// Why a file could not be read or written.
#[derive(Debug, Clone, PartialEq)]
pub enum InterchangeError {
    Io(String),
    /// The extension is not one of the formats, and the contents did not say.
    UnknownFormat(String),
    /// The file is of a known format but does not hold what it claims.
    Malformed(String),
    /// A known format, in a variant not implemented.
    Unsupported(String),
    /// Read without trouble, but there were no points in it.
    NoPoints,
}

impl fmt::Display for InterchangeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(why) => write!(f, "{why}"),
            Self::UnknownFormat(name) => write!(
                f,
                "{name} is not a scan format this reads: try .ply, .obj, .stl, .pcd, \
                 .asc, .xyz, .pts or .txt"
            ),
            Self::Malformed(why) => write!(f, "the file is damaged or not what it says: {why}"),
            Self::Unsupported(what) => write!(f, "{what}"),
            Self::NoPoints => write!(f, "the file read correctly but holds no points"),
        }
    }
}

impl std::error::Error for InterchangeError {}

type Fallible<T> = Result<T, InterchangeError>;

fn malformed(why: impl Into<String>) -> InterchangeError {
    InterchangeError::Malformed(why.into())
}

/// The formats a scene can be written as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Format {
    /// PLY, binary little-endian: the one to prefer. Holds points, colours,
    /// normals and triangles, and every 3D tool reads it.
    Ply,
    /// PLY written as text, for looking at.
    PlyText,
    /// STL, binary. Triangles only — no colour, no points without triangles.
    Stl,
    StlText,
    /// OBJ. Triangles and points as text, colours as an extension.
    Obj,
    /// PCD, the point cloud format of PCL and several scanners. Cloud only.
    Pcd,
    /// Plain columns of numbers: .asc, .xyz, .pts, .txt. Cloud only.
    Asc,
}

impl Format {
    /// The format a path's extension names, if it names one.
    pub fn of_path(path: &Path) -> Option<Self> {
        let extension = path.extension()?.to_str()?.to_ascii_lowercase();
        Some(match extension.as_str() {
            "ply" => Self::Ply,
            "stl" => Self::Stl,
            "obj" => Self::Obj,
            "pcd" => Self::Pcd,
            "asc" | "xyz" | "pts" | "txt" | "p3" => Self::Asc,
            _ => return None,
        })
    }

    /// Whether triangles survive being written in this format.
    pub fn keeps_triangles(self) -> bool {
        matches!(self, Self::Ply | Self::PlyText | Self::Stl | Self::StlText | Self::Obj)
    }

    /// Whether colours survive.
    pub fn keeps_colors(self) -> bool {
        matches!(self, Self::Ply | Self::PlyText | Self::Obj | Self::Pcd | Self::Asc)
    }

    fn extension(self) -> &'static str {
        match self {
            Self::Ply | Self::PlyText => "ply",
            Self::Stl | Self::StlText => "stl",
            Self::Obj => "obj",
            Self::Pcd => "pcd",
            Self::Asc => "asc",
        }
    }
}

/// Reads a scan file, working out the format from the name and, failing
/// that, from what is inside it.
pub fn read(path: &Path) -> Fallible<Scene> {
    let bytes = std::fs::read(path)
        .map_err(|e| InterchangeError::Io(format!("{} could not be read: {e}", path.display())))?;
    let named = Format::of_path(path);
    let format = match sniff(&bytes) {
        // What is inside wins: scanners do write .txt files full of PLY.
        Some(found) => found,
        None => named.ok_or_else(|| {
            InterchangeError::UnknownFormat(
                path.extension()
                    .and_then(|e| e.to_str())
                    .map_or_else(|| path.display().to_string(), |e| format!(".{e}")),
            )
        })?,
    };
    let mut scene = match format {
        Format::Ply | Format::PlyText => read_ply(&bytes),
        Format::Stl | Format::StlText => read_stl(&bytes),
        Format::Obj => read_obj(&bytes),
        Format::Pcd => read_pcd(&bytes),
        Format::Asc => read_asc(&bytes),
    }?;
    scene.tidy();
    if scene.is_empty() {
        return Err(InterchangeError::NoPoints);
    }
    Ok(scene)
}

/// The format a file's first bytes give away, for the ones that say so.
fn sniff(bytes: &[u8]) -> Option<Format> {
    if bytes.starts_with(b"ply") {
        return Some(Format::Ply);
    }
    if bytes.starts_with(b"solid ") && looks_like_text(bytes) {
        return Some(Format::StlText);
    }
    // A binary STL is 84 bytes of header and 50 a triangle, exactly.
    if bytes.len() >= 84 {
        let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        if bytes.len() == 84 + count * 50 && count > 0 {
            return Some(Format::Stl);
        }
    }
    let head = &bytes[..bytes.len().min(512)];
    if head.starts_with(b"# .PCD") || head.windows(8).any(|w| w == b"VERSION ") {
        return Some(Format::Pcd);
    }
    None
}

fn looks_like_text(bytes: &[u8]) -> bool {
    bytes.iter().take(512).all(|&b| b == b'\n' || b == b'\r' || b == b'\t' || (0x20..0x7f).contains(&b))
}

/// Writes a scene. The format comes from `as_format`, or from the path's
/// extension when that is None.
pub fn write(path: &Path, scene: &Scene, as_format: Option<Format>) -> Fallible<()> {
    if scene.is_empty() {
        return Err(InterchangeError::NoPoints);
    }
    let format = as_format
        .or_else(|| Format::of_path(path))
        .ok_or_else(|| InterchangeError::UnknownFormat(path.display().to_string()))?;
    if matches!(format, Format::Stl | Format::StlText) && scene.indices.is_empty() {
        return Err(InterchangeError::Unsupported(
            "STL holds triangles only, and this scan is a point cloud: mesh it first, \
             or save it as PLY"
                .into(),
        ));
    }
    let file = File::create(path)
        .map_err(|e| InterchangeError::Io(format!("{} could not be written: {e}", path.display())))?;
    let mut out = BufWriter::new(file);
    let wrote = match format {
        Format::Ply => write_ply_binary(&mut out, scene),
        Format::PlyText => write_ply_text(&mut out, scene),
        Format::Stl => write_stl_binary(&mut out, scene),
        Format::StlText => write_stl_text(&mut out, scene),
        Format::Obj => write_obj(&mut out, scene),
        Format::Pcd => write_pcd(&mut out, scene),
        Format::Asc => write_asc(&mut out, scene),
    };
    wrote.and_then(|()| {
        out.flush()
            .map_err(|e| InterchangeError::Io(format!("{} could not be finished: {e}", path.display())))
    })
}

/// The path to write to, with the format's own extension put on it.
pub fn with_extension(path: &Path, format: Format) -> std::path::PathBuf {
    path.with_extension(format.extension())
}

// ---------------------------------------------------------------- PLY

#[derive(Debug, Clone, Copy, PartialEq)]
enum Scalar {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Scalar {
    fn named(name: &str) -> Option<Self> {
        Some(match name {
            "char" | "int8" => Self::I8,
            "uchar" | "uint8" => Self::U8,
            "short" | "int16" => Self::I16,
            "ushort" | "uint16" => Self::U16,
            "int" | "int32" => Self::I32,
            "uint" | "uint32" => Self::U32,
            "float" | "float32" => Self::F32,
            "double" | "float64" => Self::F64,
            _ => return None,
        })
    }

    fn width(self) -> usize {
        match self {
            Self::I8 | Self::U8 => 1,
            Self::I16 | Self::U16 => 2,
            Self::I32 | Self::U32 | Self::F32 => 4,
            Self::F64 => 8,
        }
    }

    /// The value at the front of `bytes`, as an f64 whatever it was stored as.
    fn take(self, bytes: &[u8], at: &mut usize, big_endian: bool) -> Fallible<f64> {
        let width = self.width();
        let end = at.checked_add(width).ok_or_else(|| malformed("a value runs past the end"))?;
        let slice = bytes
            .get(*at..end)
            .ok_or_else(|| malformed("the file ends in the middle of a value"))?;
        *at = end;
        macro_rules! number {
            ($kind:ty) => {{
                let mut raw = [0u8; std::mem::size_of::<$kind>()];
                raw.copy_from_slice(slice);
                let value = if big_endian {
                    <$kind>::from_be_bytes(raw)
                } else {
                    <$kind>::from_le_bytes(raw)
                };
                value as f64
            }};
        }
        Ok(match self {
            Self::I8 => slice[0] as i8 as f64,
            Self::U8 => slice[0] as f64,
            Self::I16 => number!(i16),
            Self::U16 => number!(u16),
            Self::I32 => number!(i32),
            Self::U32 => number!(u32),
            Self::F32 => number!(f32),
            Self::F64 => number!(f64),
        })
    }
}

#[derive(Debug, Clone)]
enum Property {
    Single { name: String, kind: Scalar },
    List { name: String, count: Scalar, item: Scalar },
}

#[derive(Debug, Clone)]
struct Element {
    name: String,
    count: usize,
    properties: Vec<Property>,
}

/// Which vertex property carries what. Several spellings are in use.
fn vertex_role(name: &str) -> Option<usize> {
    Some(match name {
        "x" => 0,
        "y" => 1,
        "z" => 2,
        "nx" | "normal_x" => 3,
        "ny" | "normal_y" => 4,
        "nz" | "normal_z" => 5,
        "red" | "r" | "diffuse_red" => 6,
        "green" | "g" | "diffuse_green" => 7,
        "blue" | "b" | "diffuse_blue" => 8,
        _ => return None,
    })
}

fn read_ply(bytes: &[u8]) -> Fallible<Scene> {
    let (header, body_at) = ply_header(bytes)?;
    let (text, big_endian, elements) = header;
    if text {
        read_ply_text(&bytes[body_at..], &elements)
    } else {
        read_ply_binary(bytes, body_at, big_endian, &elements)
    }
}

type PlyHeader = (bool, bool, Vec<Element>);

fn ply_header(bytes: &[u8]) -> Fallible<(PlyHeader, usize)> {
    let mut at = 0usize;
    let mut text = None;
    let mut big_endian = false;
    let mut elements: Vec<Element> = Vec::new();
    loop {
        let line_end = bytes[at..]
            .iter()
            .position(|&b| b == b'\n')
            .ok_or_else(|| malformed("the PLY header has no end_header line"))?;
        let line = std::str::from_utf8(&bytes[at..at + line_end])
            .map_err(|_| malformed("the PLY header is not text"))?
            .trim_end_matches('\r')
            .trim()
            .to_owned();
        at += line_end + 1;
        let mut words = line.split_whitespace();
        match words.next() {
            None | Some("ply") | Some("comment") | Some("obj_info") => {}
            Some("format") => match words.next() {
                Some("ascii") => text = Some(true),
                Some("binary_little_endian") => text = Some(false),
                Some("binary_big_endian") => {
                    text = Some(false);
                    big_endian = true;
                }
                other => {
                    return Err(InterchangeError::Unsupported(format!(
                        "PLY in {} is not a format this reads",
                        other.unwrap_or("no")
                    )))
                }
            },
            Some("element") => {
                let name = words.next().unwrap_or_default().to_owned();
                let count = words
                    .next()
                    .and_then(|n| n.parse::<usize>().ok())
                    .ok_or_else(|| malformed(format!("element {name} has no count")))?;
                elements.push(Element { name, count, properties: Vec::new() });
            }
            Some("property") => {
                let element = elements
                    .last_mut()
                    .ok_or_else(|| malformed("a property before any element"))?;
                let first = words.next().unwrap_or_default();
                if first == "list" {
                    let count = Scalar::named(words.next().unwrap_or_default())
                        .ok_or_else(|| malformed("a list property with no count type"))?;
                    let item = Scalar::named(words.next().unwrap_or_default())
                        .ok_or_else(|| malformed("a list property with no item type"))?;
                    let name = words.next().unwrap_or_default().to_owned();
                    element.properties.push(Property::List { name, count, item });
                } else {
                    let kind = Scalar::named(first)
                        .ok_or_else(|| malformed(format!("{first} is not a PLY type")))?;
                    let name = words.next().unwrap_or_default().to_owned();
                    element.properties.push(Property::Single { name, kind });
                }
            }
            Some("end_header") => break,
            Some(_) => {} // a line this does not need
        }
    }
    let text = text.ok_or_else(|| malformed("the PLY header does not say which format it is"))?;
    Ok(((text, big_endian, elements), at))
}

/// Colour as stored: 0-255 whole numbers, or 0-1 fractions in a float.
fn to_byte(value: f64, kind: Scalar) -> u8 {
    let scaled = if matches!(kind, Scalar::F32 | Scalar::F64) && (0.0..=1.0).contains(&value) {
        value * 255.0
    } else {
        value
    };
    scaled.clamp(0.0, 255.0).round() as u8
}

fn read_ply_binary(
    bytes: &[u8],
    mut at: usize,
    big_endian: bool,
    elements: &[Element],
) -> Fallible<Scene> {
    let mut scene = Scene::default();
    for element in elements {
        let vertices = element.name == "vertex";
        let faces = element.name == "face" || element.name == "tristrips";
        if vertices {
            scene.points.reserve(element.count);
        }
        for _ in 0..element.count {
            let mut slot = [f64::NAN; 9];
            let mut color_kind = Scalar::U8;
            for property in &element.properties {
                match property {
                    Property::Single { name, kind } => {
                        let value = kind.take(bytes, &mut at, big_endian)?;
                        if vertices {
                            if let Some(role) = vertex_role(name) {
                                slot[role] = value;
                                if role >= 6 {
                                    color_kind = *kind;
                                }
                            }
                        }
                    }
                    Property::List { name, count, item } => {
                        let n = count.take(bytes, &mut at, big_endian)? as usize;
                        let mut corners = Vec::with_capacity(n.min(64));
                        for _ in 0..n {
                            corners.push(item.take(bytes, &mut at, big_endian)? as i64);
                        }
                        if faces && (name == "vertex_indices" || name == "vertex_index") {
                            push_face(&mut scene.indices, &corners)?;
                        }
                    }
                }
            }
            if vertices {
                take_vertex(&mut scene, &slot, color_kind);
            }
        }
    }
    Ok(scene)
}

fn read_ply_text(body: &[u8], elements: &[Element]) -> Fallible<Scene> {
    let text = std::str::from_utf8(body).map_err(|_| malformed("the PLY body is not text"))?;
    let mut lines = text.lines().filter(|l| !l.trim().is_empty());
    let mut scene = Scene::default();
    for element in elements {
        let vertices = element.name == "vertex";
        let faces = element.name == "face" || element.name == "tristrips";
        for _ in 0..element.count {
            let line = lines.next().ok_or_else(|| {
                malformed(format!("the file ends before its {} are all there", element.name))
            })?;
            let mut words = line.split_whitespace();
            let mut slot = [f64::NAN; 9];
            let mut color_kind = Scalar::U8;
            for property in &element.properties {
                match property {
                    Property::Single { name, kind } => {
                        let value: f64 = words
                            .next()
                            .and_then(|w| w.parse().ok())
                            .ok_or_else(|| malformed(format!("{name} is missing or not a number")))?;
                        if vertices {
                            if let Some(role) = vertex_role(name) {
                                slot[role] = value;
                                if role >= 6 {
                                    color_kind = *kind;
                                }
                            }
                        }
                    }
                    Property::List { name, .. } => {
                        let n: usize = words
                            .next()
                            .and_then(|w| w.parse().ok())
                            .ok_or_else(|| malformed("a list with no count"))?;
                        let mut corners = Vec::with_capacity(n.min(64));
                        for _ in 0..n {
                            corners.push(
                                words
                                    .next()
                                    .and_then(|w| w.parse::<i64>().ok())
                                    .ok_or_else(|| malformed("a list is shorter than its count"))?,
                            );
                        }
                        if faces && (name == "vertex_indices" || name == "vertex_index") {
                            push_face(&mut scene.indices, &corners)?;
                        }
                    }
                }
            }
            if vertices {
                take_vertex(&mut scene, &slot, color_kind);
            }
        }
    }
    Ok(scene)
}

fn take_vertex(scene: &mut Scene, slot: &[f64; 9], color_kind: Scalar) {
    scene
        .points
        .push(Point3::new(slot[0] as f32, slot[1] as f32, slot[2] as f32));
    if slot[3..6].iter().all(|v| v.is_finite()) {
        scene
            .normals
            .push(Vector3::new(slot[3] as f32, slot[4] as f32, slot[5] as f32));
    }
    if slot[6..9].iter().all(|v| v.is_finite()) {
        scene.colors.push([
            to_byte(slot[6], color_kind),
            to_byte(slot[7], color_kind),
            to_byte(slot[8], color_kind),
        ]);
    }
}

/// A face of any number of corners, as a fan of triangles.
fn push_face(indices: &mut Vec<u32>, corners: &[i64]) -> Fallible<()> {
    if corners.len() < 3 {
        return Ok(()); // a point or an edge: nothing to draw
    }
    for corner in corners {
        if *corner < 0 || *corner > u32::MAX as i64 {
            return Err(malformed("a face points at a vertex that cannot exist"));
        }
    }
    for n in 1..corners.len() - 1 {
        indices.push(corners[0] as u32);
        indices.push(corners[n] as u32);
        indices.push(corners[n + 1] as u32);
    }
    Ok(())
}

fn io<T>(result: std::io::Result<T>) -> Fallible<T> {
    result.map_err(|e| InterchangeError::Io(e.to_string()))
}

fn ply_header_text(scene: &Scene, format: &str) -> String {
    let mut header = format!(
        "ply\nformat {format} 1.0\ncomment written by All3DScan (millimetres)\n\
         element vertex {}\nproperty float x\nproperty float y\nproperty float z\n",
        scene.points.len()
    );
    if scene.has_normals() {
        header.push_str("property float nx\nproperty float ny\nproperty float nz\n");
    }
    if scene.has_colors() {
        header.push_str("property uchar red\nproperty uchar green\nproperty uchar blue\n");
    }
    if !scene.indices.is_empty() {
        header.push_str(&format!(
            "element face {}\nproperty list uchar int vertex_indices\n",
            scene.triangles()
        ));
    }
    header.push_str("end_header\n");
    header
}

fn write_ply_binary(out: &mut impl Write, scene: &Scene) -> Fallible<()> {
    io(out.write_all(ply_header_text(scene, "binary_little_endian").as_bytes()))?;
    let (normals, colors) = (scene.has_normals(), scene.has_colors());
    let mut row = Vec::with_capacity(4 * 6 + 3);
    for (at, point) in scene.points.iter().enumerate() {
        row.clear();
        for axis in 0..3 {
            row.extend_from_slice(&point[axis].to_le_bytes());
        }
        if normals {
            for axis in 0..3 {
                row.extend_from_slice(&scene.normals[at][axis].to_le_bytes());
            }
        }
        if colors {
            row.extend_from_slice(&scene.colors[at]);
        }
        io(out.write_all(&row))?;
    }
    for triangle in scene.indices.chunks_exact(3) {
        row.clear();
        row.push(3u8);
        for corner in triangle {
            row.extend_from_slice(&corner.to_le_bytes());
        }
        io(out.write_all(&row))?;
    }
    Ok(())
}

fn write_ply_text(out: &mut impl Write, scene: &Scene) -> Fallible<()> {
    io(out.write_all(ply_header_text(scene, "ascii").as_bytes()))?;
    let (normals, colors) = (scene.has_normals(), scene.has_colors());
    for (at, point) in scene.points.iter().enumerate() {
        let mut line = format!("{:.6} {:.6} {:.6}", point.x, point.y, point.z);
        if normals {
            let n = scene.normals[at];
            line.push_str(&format!(" {:.6} {:.6} {:.6}", n.x, n.y, n.z));
        }
        if colors {
            let c = scene.colors[at];
            line.push_str(&format!(" {} {} {}", c[0], c[1], c[2]));
        }
        line.push('\n');
        io(out.write_all(line.as_bytes()))?;
    }
    for triangle in scene.indices.chunks_exact(3) {
        io(out.write_all(
            format!("3 {} {} {}\n", triangle[0], triangle[1], triangle[2]).as_bytes(),
        ))?;
    }
    Ok(())
}

// ---------------------------------------------------------------- STL

fn read_stl(bytes: &[u8]) -> Fallible<Scene> {
    // The ascii form starts "solid", but so do some binary files written by
    // careless tools, so the exact binary length decides first.
    if bytes.len() >= 84 {
        let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        if bytes.len() == 84 + count * 50 {
            return read_stl_binary(bytes, count);
        }
    }
    if bytes.starts_with(b"solid") {
        return read_stl_text(bytes);
    }
    Err(malformed("an STL that is neither the binary nor the text form"))
}

/// Triangle corners welded into a vertex list, so a mesh of N triangles does
/// not come out as 3N unconnected vertices. Exact bits have to match, which
/// is what every STL writer produces for a shared corner.
struct Weld {
    seen: HashMap<[u32; 3], u32>,
    scene: Scene,
}

impl Weld {
    fn new() -> Self {
        Self { seen: HashMap::new(), scene: Scene::default() }
    }

    fn corner(&mut self, x: f32, y: f32, z: f32) {
        let key = [x.to_bits(), y.to_bits(), z.to_bits()];
        let at = *self.seen.entry(key).or_insert_with(|| {
            self.scene.points.push(Point3::new(x, y, z));
            (self.scene.points.len() - 1) as u32
        });
        self.scene.indices.push(at);
    }
}

fn read_stl_binary(bytes: &[u8], count: usize) -> Fallible<Scene> {
    let mut weld = Weld::new();
    let mut at = 84usize;
    let number = |bytes: &[u8], at: usize| {
        f32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
    };
    for _ in 0..count {
        // 12 bytes of facet normal, which is recomputed per vertex later.
        for corner in 0..3 {
            let base = at + 12 + corner * 12;
            weld.corner(number(bytes, base), number(bytes, base + 4), number(bytes, base + 8));
        }
        at += 50;
    }
    Ok(weld.scene)
}

fn read_stl_text(bytes: &[u8]) -> Fallible<Scene> {
    let text = std::str::from_utf8(bytes).map_err(|_| malformed("the STL is not text"))?;
    let mut weld = Weld::new();
    for line in text.lines() {
        let mut words = line.split_whitespace();
        if words.next() != Some("vertex") {
            continue;
        }
        let mut corner = [0f32; 3];
        for (axis, slot) in corner.iter_mut().enumerate() {
            *slot = words
                .next()
                .and_then(|w| w.parse().ok())
                .ok_or_else(|| malformed(format!("a vertex is missing its {} coordinate", "xyz".as_bytes()[axis] as char)))?;
        }
        weld.corner(corner[0], corner[1], corner[2]);
    }
    if weld.scene.indices.len() % 3 != 0 {
        return Err(malformed("the STL ends part way through a triangle"));
    }
    Ok(weld.scene)
}

/// A triangle's own normal, for the formats that store one per face.
fn facet_normal(scene: &Scene, triangle: &[u32]) -> [f32; 3] {
    let (a, b, c) = (
        scene.points[triangle[0] as usize],
        scene.points[triangle[1] as usize],
        scene.points[triangle[2] as usize],
    );
    let normal = (b - a).cross(&(c - a));
    normal.try_normalize(1e-12).map_or([0.0, 0.0, 0.0], |n| [n.x, n.y, n.z])
}

fn write_stl_binary(out: &mut impl Write, scene: &Scene) -> Fallible<()> {
    let mut header = [0u8; 80];
    let note = b"All3DScan (millimetres)";
    header[..note.len()].copy_from_slice(note);
    io(out.write_all(&header))?;
    io(out.write_all(&(scene.triangles() as u32).to_le_bytes()))?;
    for triangle in scene.indices.chunks_exact(3) {
        let mut row = Vec::with_capacity(50);
        for part in facet_normal(scene, triangle) {
            row.extend_from_slice(&part.to_le_bytes());
        }
        for corner in triangle {
            let point = scene.points[*corner as usize];
            for axis in 0..3 {
                row.extend_from_slice(&point[axis].to_le_bytes());
            }
        }
        row.extend_from_slice(&0u16.to_le_bytes()); // attribute byte count
        io(out.write_all(&row))?;
    }
    Ok(())
}

fn write_stl_text(out: &mut impl Write, scene: &Scene) -> Fallible<()> {
    io(out.write_all(b"solid all3dscan\n"))?;
    for triangle in scene.indices.chunks_exact(3) {
        let n = facet_normal(scene, triangle);
        io(out.write_all(
            format!("facet normal {:e} {:e} {:e}\n  outer loop\n", n[0], n[1], n[2]).as_bytes(),
        ))?;
        for corner in triangle {
            let p = scene.points[*corner as usize];
            io(out.write_all(format!("    vertex {:e} {:e} {:e}\n", p.x, p.y, p.z).as_bytes()))?;
        }
        io(out.write_all(b"  endloop\nendfacet\n"))?;
    }
    io(out.write_all(b"endsolid all3dscan\n"))?;
    Ok(())
}

// ---------------------------------------------------------------- OBJ

fn read_obj(bytes: &[u8]) -> Fallible<Scene> {
    let text = std::str::from_utf8(bytes).map_err(|_| malformed("the OBJ is not text"))?;
    let mut scene = Scene::default();
    let mut normals: Vec<Vector3<f32>> = Vec::new();
    // Which normal each vertex was used with, filled in as faces name them.
    let mut per_vertex: Vec<Option<usize>> = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        let mut words = line.split_whitespace();
        match words.next() {
            Some("v") => {
                let parts: Vec<f32> = words.filter_map(|w| w.parse().ok()).collect();
                if parts.len() < 3 {
                    return Err(malformed("a vertex with fewer than three coordinates"));
                }
                scene.points.push(Point3::new(parts[0], parts[1], parts[2]));
                per_vertex.push(None);
                // Some writers put the colour after the position.
                if parts.len() >= 6 {
                    scene.colors.push([
                        to_byte(parts[3] as f64, Scalar::F32),
                        to_byte(parts[4] as f64, Scalar::F32),
                        to_byte(parts[5] as f64, Scalar::F32),
                    ]);
                }
            }
            Some("vn") => {
                let parts: Vec<f32> = words.filter_map(|w| w.parse().ok()).collect();
                if parts.len() >= 3 {
                    normals.push(Vector3::new(parts[0], parts[1], parts[2]));
                }
            }
            Some("f") => {
                let mut corners = Vec::new();
                for word in words {
                    let mut fields = word.split('/');
                    let vertex = fields.next().unwrap_or_default();
                    let Some(at) = obj_index(vertex, scene.points.len()) else {
                        return Err(malformed(format!("{word} is not a face corner")));
                    };
                    let normal_field = fields.nth(1).unwrap_or_default();
                    if let Some(n) = obj_index(normal_field, normals.len()) {
                        if at < per_vertex.len() {
                            per_vertex[at] = Some(n);
                        }
                    }
                    corners.push(at as i64);
                }
                push_face(&mut scene.indices, &corners)?;
            }
            _ => {}
        }
    }
    if !normals.is_empty() && per_vertex.iter().any(Option::is_some) {
        scene.normals = per_vertex
            .iter()
            .map(|at| at.and_then(|n| normals.get(n).copied()).unwrap_or_else(Vector3::zeros))
            .collect();
    } else if normals.len() == scene.points.len() {
        // A cloud written with a normal per point and no faces at all.
        scene.normals = normals;
    }
    Ok(scene)
}

/// An OBJ index: 1-based, and negative counts back from the end.
fn obj_index(field: &str, so_far: usize) -> Option<usize> {
    let number: i64 = field.parse().ok()?;
    let at = if number > 0 {
        number - 1
    } else if number < 0 {
        so_far as i64 + number
    } else {
        return None;
    };
    (at >= 0 && (at as usize) < so_far).then_some(at as usize)
}

fn write_obj(out: &mut impl Write, scene: &Scene) -> Fallible<()> {
    io(out.write_all(b"# written by All3DScan (millimetres)\n"))?;
    let colors = scene.has_colors();
    for (at, point) in scene.points.iter().enumerate() {
        let mut line = format!("v {:.6} {:.6} {:.6}", point.x, point.y, point.z);
        if colors {
            let c = scene.colors[at];
            line.push_str(&format!(
                " {:.6} {:.6} {:.6}",
                f32::from(c[0]) / 255.0,
                f32::from(c[1]) / 255.0,
                f32::from(c[2]) / 255.0
            ));
        }
        line.push('\n');
        io(out.write_all(line.as_bytes()))?;
    }
    if scene.has_normals() {
        for normal in &scene.normals {
            io(out.write_all(
                format!("vn {:.6} {:.6} {:.6}\n", normal.x, normal.y, normal.z).as_bytes(),
            ))?;
        }
    }
    let normals = scene.has_normals();
    for triangle in scene.indices.chunks_exact(3) {
        let corner = |c: u32| {
            let one_based = c + 1;
            if normals {
                format!("{one_based}//{one_based}")
            } else {
                one_based.to_string()
            }
        };
        io(out.write_all(
            format!(
                "f {} {} {}\n",
                corner(triangle[0]),
                corner(triangle[1]),
                corner(triangle[2])
            )
            .as_bytes(),
        ))?;
    }
    Ok(())
}

// ---------------------------------------------------------------- PCD

fn read_pcd(bytes: &[u8]) -> Fallible<Scene> {
    let mut fields: Vec<String> = Vec::new();
    let mut sizes: Vec<usize> = Vec::new();
    let mut types: Vec<char> = Vec::new();
    let mut counts: Vec<usize> = Vec::new();
    let mut points = 0usize;
    let data: String;
    let mut at = 0usize;
    let mut reader = BufReader::new(bytes);
    let mut line = String::new();
    loop {
        line.clear();
        let read = io(reader.read_line(&mut line))?;
        if read == 0 {
            return Err(malformed("the PCD header has no DATA line"));
        }
        at += read;
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let mut words = trimmed.split_whitespace();
        let key = words.next().unwrap_or_default().to_ascii_uppercase();
        match key.as_str() {
            "FIELDS" => fields = words.map(str::to_owned).collect(),
            "SIZE" => sizes = words.filter_map(|w| w.parse().ok()).collect(),
            "TYPE" => types = words.filter_map(|w| w.chars().next()).collect(),
            "COUNT" => counts = words.filter_map(|w| w.parse().ok()).collect(),
            "POINTS" => points = words.next().and_then(|w| w.parse().ok()).unwrap_or(0),
            "WIDTH" if points == 0 => {
                points = words.next().and_then(|w| w.parse().ok()).unwrap_or(0);
            }
            "DATA" => {
                data = words.next().unwrap_or("ascii").to_ascii_lowercase();
                break;
            }
            _ => {}
        }
    }
    if data == "binary_compressed" {
        return Err(InterchangeError::Unsupported(
            "this PCD is binary_compressed, which this does not read: save it as \
             binary or ascii PCD, or as PLY"
                .into(),
        ));
    }
    if counts.len() != fields.len() {
        counts = vec![1; fields.len()];
    }
    if sizes.len() != fields.len() || types.len() != fields.len() {
        return Err(malformed("the PCD header's FIELDS, SIZE and TYPE do not agree"));
    }
    let kinds: Vec<Scalar> = fields
        .iter()
        .enumerate()
        .map(|(n, _)| match (types[n], sizes[n]) {
            ('F', 4) => Ok(Scalar::F32),
            ('F', 8) => Ok(Scalar::F64),
            ('U', 1) => Ok(Scalar::U8),
            ('U', 2) => Ok(Scalar::U16),
            ('U', 4) => Ok(Scalar::U32),
            ('I', 1) => Ok(Scalar::I8),
            ('I', 2) => Ok(Scalar::I16),
            ('I', 4) => Ok(Scalar::I32),
            (t, s) => Err(malformed(format!("a PCD field of type {t} and size {s}"))),
        })
        .collect::<Fallible<_>>()?;

    let role: Vec<Option<usize>> = fields
        .iter()
        .map(|f| match f.as_str() {
            "x" => Some(0),
            "y" => Some(1),
            "z" => Some(2),
            "normal_x" => Some(3),
            "normal_y" => Some(4),
            "normal_z" => Some(5),
            "rgb" | "rgba" => Some(9), // one packed number, unpacked below
            "r" | "red" => Some(6),
            "g" | "green" => Some(7),
            "b" | "blue" => Some(8),
            _ => None,
        })
        .collect();

    let mut scene = Scene::default();
    let mut take = |slot: &mut [f64; 10]| {
        scene.points.push(Point3::new(slot[0] as f32, slot[1] as f32, slot[2] as f32));
        if slot[3..6].iter().all(|v| v.is_finite()) {
            scene
                .normals
                .push(Vector3::new(slot[3] as f32, slot[4] as f32, slot[5] as f32));
        }
        if slot[9].is_finite() {
            let packed = (slot[9] as f32).to_bits();
            scene.colors.push([
                ((packed >> 16) & 0xff) as u8,
                ((packed >> 8) & 0xff) as u8,
                (packed & 0xff) as u8,
            ]);
        } else if slot[6..9].iter().all(|v| v.is_finite()) {
            scene.colors.push([
                to_byte(slot[6], Scalar::U8),
                to_byte(slot[7], Scalar::U8),
                to_byte(slot[8], Scalar::U8),
            ]);
        }
    };

    if data == "ascii" {
        let text = std::str::from_utf8(&bytes[at..]).map_err(|_| malformed("the PCD body is not text"))?;
        for line in text.lines().filter(|l| !l.trim().is_empty()) {
            let mut words = line.split_whitespace();
            let mut slot = [f64::NAN; 10];
            for (n, kind) in kinds.iter().enumerate() {
                for repeat in 0..counts[n] {
                    let Some(word) = words.next() else { break };
                    let value: f64 = word.parse().unwrap_or(f64::NAN);
                    if repeat == 0 {
                        if let Some(r) = role[n] {
                            // A packed colour is written as the decimal of a
                            // float whose *bits* are the colour, so it is
                            // kept as the number it reads as and unpacked
                            // with the binary form's, in take().
                            slot[r] = value;
                        }
                    }
                    let _ = kind;
                }
            }
            take(&mut slot);
        }
    } else {
        let stride: usize = kinds.iter().zip(&counts).map(|(k, c)| k.width() * c).sum();
        if stride == 0 {
            return Err(malformed("a PCD whose points are zero bytes long"));
        }
        let available = (bytes.len() - at) / stride;
        for n in 0..points.min(available) {
            let mut cursor = at + n * stride;
            let mut slot = [f64::NAN; 10];
            for (field, kind) in kinds.iter().enumerate() {
                for repeat in 0..counts[field] {
                    let value = kind.take(bytes, &mut cursor, false)?;
                    if repeat == 0 {
                        if let Some(r) = role[field] {
                            slot[r] = value;
                        }
                    }
                }
            }
            take(&mut slot);
        }
    }
    Ok(scene)
}

fn write_pcd(out: &mut impl Write, scene: &Scene) -> Fallible<()> {
    let normals = scene.has_normals();
    let colors = scene.has_colors();
    let mut fields = vec!["x", "y", "z"];
    if normals {
        fields.extend_from_slice(&["normal_x", "normal_y", "normal_z"]);
    }
    if colors {
        fields.push("rgb");
    }
    let header = format!(
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS {}\n\
         SIZE{}\nTYPE{}\nCOUNT{}\nWIDTH {}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n\
         POINTS {}\nDATA ascii\n",
        fields.join(" "),
        " 4".repeat(fields.len()),
        " F".repeat(fields.len()),
        " 1".repeat(fields.len()),
        scene.points.len(),
        scene.points.len()
    );
    io(out.write_all(header.as_bytes()))?;
    for (at, point) in scene.points.iter().enumerate() {
        let mut line = format!("{:.6} {:.6} {:.6}", point.x, point.y, point.z);
        if normals {
            let n = scene.normals[at];
            line.push_str(&format!(" {:.6} {:.6} {:.6}", n.x, n.y, n.z));
        }
        if colors {
            let c = scene.colors[at];
            let packed = (u32::from(c[0]) << 16) | (u32::from(c[1]) << 8) | u32::from(c[2]);
            line.push_str(&format!(" {}", f32::from_bits(packed)));
        }
        line.push('\n');
        io(out.write_all(line.as_bytes()))?;
    }
    Ok(())
}

// ---------------------------------------------- plain columns of numbers

/// Where in a row the normal and the colour start, when they are there at
/// all. Columns beyond the position are intensity, quality, a scan index and
/// whatever else a writer felt like adding, which this skips.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct Layout {
    normals_at: Option<usize>,
    colors_at: Option<usize>,
}

/// Reads whitespace-separated columns: .asc, .xyz, .pts, .txt and .p3.
///
/// Which columns mean what is not written down anywhere in these files, so
/// it is worked out from the numbers themselves: colours are whole numbers
/// from 0 to 255, normals are unit length. Guessing wrong on a cloud's
/// colours is harmless; guessing wrong on its positions is not, and the
/// first three columns are always the position.
fn read_asc(bytes: &[u8]) -> Fallible<Scene> {
    let text = std::str::from_utf8(bytes).map_err(|_| malformed("the file is not text"))?;
    let rows: Vec<Vec<f64>> = text
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with(['#', '*', '/', ';']))
        .map(|line| {
            line.split(|c: char| c.is_whitespace() || c == ',')
                .filter(|w| !w.is_empty())
                .map(|w| w.parse::<f64>().unwrap_or(f64::NAN))
                .collect()
        })
        .filter(|row: &Vec<f64>| row.len() >= 3 && row[..3].iter().all(|v| v.is_finite()))
        .collect();
    if rows.is_empty() {
        return Err(InterchangeError::NoPoints);
    }
    let columns = rows.iter().map(Vec::len).min().unwrap_or(0);
    let layout = sniff_columns(&rows, columns);
    let mut scene = Scene::default();
    scene.points.reserve(rows.len());
    for row in &rows {
        scene.points.push(Point3::new(row[0] as f32, row[1] as f32, row[2] as f32));
        if let Some(at) = layout.normals_at {
            scene
                .normals
                .push(Vector3::new(row[at] as f32, row[at + 1] as f32, row[at + 2] as f32));
        }
        if let Some(at) = layout.colors_at {
            scene.colors.push([
                to_byte(row[at], Scalar::U8),
                to_byte(row[at + 1], Scalar::U8),
                to_byte(row[at + 2], Scalar::U8),
            ]);
        }
    }
    Ok(scene)
}

fn sniff_columns(rows: &[Vec<f64>], columns: usize) -> Layout {
    let sample = rows.len().min(200);
    let looks_like_color = |from: usize| {
        columns > from + 2
            && rows[..sample].iter().all(|row| {
                row[from..from + 3]
                    .iter()
                    .all(|v| v.is_finite() && *v >= 0.0 && *v <= 255.0 && v.fract() == 0.0)
            })
            // All-zero or all-one columns are more likely a flag than a colour.
            && rows[..sample]
                .iter()
                .any(|row| row[from..from + 3].iter().any(|v| *v > 1.0))
    };
    let looks_like_normals = |from: usize| {
        columns > from + 2
            && rows[..sample].iter().all(|row| {
                let v = Vector3::new(row[from], row[from + 1], row[from + 2]);
                v.iter().all(|c| c.is_finite()) && (v.norm() - 1.0).abs() < 0.05
            })
    };
    let normals_at = looks_like_normals(3).then_some(3);
    // With a normal in the way the colour sits at 6, or at 7 behind an
    // intensity; without one it sits at 3, or at 4 behind an intensity. The
    // later position is tried first, since the earlier one would otherwise
    // match the tail of the longer layout.
    let wherever: &[usize] = if normals_at.is_some() { &[7, 6] } else { &[4, 3] };
    let colors_at = wherever.iter().copied().find(|&from| looks_like_color(from));
    Layout { normals_at, colors_at }
}

fn write_asc(out: &mut impl Write, scene: &Scene) -> Fallible<()> {
    let normals = scene.has_normals();
    let colors = scene.has_colors();
    for (at, point) in scene.points.iter().enumerate() {
        let mut line = format!("{:.6} {:.6} {:.6}", point.x, point.y, point.z);
        if normals {
            let n = scene.normals[at];
            line.push_str(&format!(" {:.6} {:.6} {:.6}", n.x, n.y, n.z));
        }
        if colors {
            let c = scene.colors[at];
            line.push_str(&format!(" {} {} {}", c[0], c[1], c[2]));
        }
        line.push('\n');
        io(out.write_all(line.as_bytes()))?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cloud() -> Scene {
        Scene {
            points: vec![
                Point3::new(1.0, 2.0, 3.0),
                Point3::new(-4.5, 5.25, 6.125),
                Point3::new(7.0, 8.0, 9.0),
            ],
            colors: vec![[255, 0, 0], [0, 128, 0], [10, 20, 30]],
            normals: vec![Vector3::x(), Vector3::y(), Vector3::z()],
            indices: Vec::new(),
        }
    }

    fn mesh() -> Scene {
        let mut scene = cloud();
        scene.points.push(Point3::new(0.0, 0.0, 0.0));
        scene.colors.push([1, 2, 3]);
        scene.normals.push(-Vector3::z());
        scene.indices = vec![0, 1, 2, 0, 2, 3];
        scene
    }

    /// Writes a scene and reads it back, through a real file.
    fn round_trip(scene: &Scene, format: Format) -> Scene {
        let dir = std::env::temp_dir().join(format!(
            "all3dscan-interchange-{}-{:?}",
            std::process::id(),
            format
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let path = with_extension(&dir.join("scene"), format);
        write(&path, scene, Some(format)).unwrap();
        let back = read(&path).unwrap();
        std::fs::remove_dir_all(&dir).ok();
        back
    }

    fn near(a: &Scene, b: &Scene) {
        assert_eq!(a.points.len(), b.points.len(), "different point counts");
        for (one, two) in a.points.iter().zip(&b.points) {
            assert!((one - two).norm() < 1e-4, "{one:?} became {two:?}");
        }
    }

    #[test]
    fn a_cloud_survives_every_format_that_holds_one() {
        let scene = cloud();
        for format in [Format::Ply, Format::PlyText, Format::Obj, Format::Pcd, Format::Asc] {
            let back = round_trip(&scene, format);
            near(&scene, &back);
            assert_eq!(back.colors, scene.colors, "{format:?} lost the colours");
            assert!(back.has_normals(), "{format:?} lost the normals");
        }
    }

    #[test]
    fn a_mesh_survives_every_format_that_holds_triangles() {
        let scene = mesh();
        for format in [Format::Ply, Format::PlyText, Format::Obj, Format::Stl, Format::StlText] {
            let back = round_trip(&scene, format);
            assert_eq!(
                back.triangles(),
                scene.triangles(),
                "{format:?} came back with the wrong triangle count"
            );
            // STL keeps no vertex list of its own, so the points are welded
            // back together from the corners; the shape has to match, not
            // the order.
            assert_eq!(back.points.len(), scene.points.len(), "{format:?} changed the vertices");
        }
    }

    #[test]
    fn a_point_cloud_will_not_be_written_as_stl() {
        let dir = std::env::temp_dir().join(format!("all3dscan-stl-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let err = write(&dir.join("c.stl"), &cloud(), None).unwrap_err();
        std::fs::remove_dir_all(&dir).ok();
        assert!(
            matches!(err, InterchangeError::Unsupported(ref why) if why.contains("mesh it first")),
            "{err}"
        );
    }

    #[test]
    fn a_ply_from_another_tool_reads() {
        // Properties in an unusual order, with ones this does not know, and
        // colours as a float: all of which real exporters write.
        let text = "ply\nformat ascii 1.0\ncomment made elsewhere\n\
                    element vertex 2\nproperty double z\nproperty double y\nproperty double x\n\
                    property float quality\nproperty float red\nproperty float green\n\
                    property float blue\nelement face 1\n\
                    property list uchar int vertex_indices\nend_header\n\
                    3 2 1 0.5 1.0 0.0 0.0\n6 5 4 0.25 0.0 1.0 0.0\n3 0 1 0\n";
        let dir = std::env::temp_dir().join(format!("all3dscan-ply-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("other.ply");
        std::fs::write(&path, text).unwrap();
        let scene = read(&path).unwrap();
        std::fs::remove_dir_all(&dir).ok();
        assert_eq!(scene.points, vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)]);
        assert_eq!(scene.colors, vec![[255, 0, 0], [0, 255, 0]]);
        assert_eq!(scene.indices, vec![0, 1, 0]);
    }

    #[test]
    fn a_face_of_more_than_three_corners_becomes_triangles() {
        let mut indices = Vec::new();
        push_face(&mut indices, &[0, 1, 2, 3]).unwrap();
        assert_eq!(indices, vec![0, 1, 2, 0, 2, 3]);
    }

    #[test]
    fn a_face_pointing_past_the_vertices_is_refused_not_trusted() {
        let text = "ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\n\
                    property float y\nproperty float z\nelement face 1\n\
                    property list uchar int vertex_indices\nend_header\n0 0 0\n3 0 9 4\n";
        let dir = std::env::temp_dir().join(format!("all3dscan-bad-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("bad.ply");
        std::fs::write(&path, text).unwrap();
        let scene = read(&path).unwrap();
        std::fs::remove_dir_all(&dir).ok();
        assert!(scene.indices.is_empty(), "a face past the end should be dropped");
        assert_eq!(scene.points.len(), 1);
    }

    #[test]
    fn the_columns_of_a_plain_text_cloud_are_worked_out() {
        let cases: Vec<(&str, usize, bool, bool)> = vec![
            ("1 2 3\n4 5 6\n", 2, false, false),
            ("1 2 3 1 0 0\n4 5 6 0 1 0\n", 2, true, false), // unit vectors: normals
            ("1 2 3 255 0 0\n4 5 6 12 200 7\n", 2, false, true), // 0-255 whole: colours
            // Shining3D's .p3: x y z nx ny nz intensity, with its own header.
            ("*****\n# rows 2\n# columns 7\n*****\n\
              -58.9 -59.6 222.7 0.0 0.0 -1.0 6.0 \n1.0 2.0 3.0 0.0 1.0 0.0 6.0 \n", 2, true, false),
            // .pts: x y z intensity r g b
            ("1 2 3 -1500 255 0 0\n4 5 6 -1400 0 255 0\n", 2, false, true),
        ];
        for (text, points, normals, colors) in cases {
            let scene = read_asc(text.as_bytes()).unwrap();
            assert_eq!(scene.points.len(), points, "{text:?}");
            assert_eq!(scene.has_normals(), normals, "normals in {text:?}");
            assert_eq!(scene.has_colors(), colors, "colours in {text:?}");
        }
    }

    #[test]
    fn a_file_is_read_by_what_is_in_it_when_the_name_misleads() {
        let dir = std::env::temp_dir().join(format!("all3dscan-sniff-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("actually.txt"); // named .txt, holding PLY
        write(&path.with_extension("ply"), &cloud(), Some(Format::PlyText)).unwrap();
        std::fs::rename(path.with_extension("ply"), &path).unwrap();
        let scene = read(&path).unwrap();
        std::fs::remove_dir_all(&dir).ok();
        near(&cloud(), &scene);
    }

    #[test]
    fn a_binary_ply_in_the_other_endianness_reads() {
        let mut bytes = b"ply\nformat binary_big_endian 1.0\nelement vertex 1\n\
                          property float x\nproperty float y\nproperty float z\nend_header\n"
            .to_vec();
        for value in [1.0f32, 2.0, 3.0] {
            bytes.extend_from_slice(&value.to_be_bytes());
        }
        let scene = read_ply(&bytes).unwrap();
        assert_eq!(scene.points, vec![Point3::new(1.0, 2.0, 3.0)]);
    }

    #[test]
    fn a_file_that_is_not_a_scan_says_so_rather_than_guessing() {
        let dir = std::env::temp_dir().join(format!("all3dscan-junk-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("notes.pdf");
        std::fs::write(&path, b"%PDF-1.7\nnot a scan at all").unwrap();
        let err = read(&path).unwrap_err();
        std::fs::remove_dir_all(&dir).ok();
        assert!(matches!(err, InterchangeError::UnknownFormat(_)), "{err}");
    }

    #[test]
    fn an_empty_file_is_refused_rather_than_opened_as_nothing() {
        let dir = std::env::temp_dir().join(format!("all3dscan-empty-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("empty.asc");
        std::fs::write(&path, b"# nothing here\n").unwrap();
        let err = read(&path).unwrap_err();
        std::fs::remove_dir_all(&dir).ok();
        assert_eq!(err, InterchangeError::NoPoints);
    }

    #[test]
    fn the_extent_tells_millimetres_from_metres() {
        let (low, high) = cloud().extent().unwrap();
        assert_eq!(low, Point3::new(-4.5, 2.0, 3.0));
        assert_eq!(high, Point3::new(7.0, 8.0, 9.0));
        let mut small = cloud();
        small.scale(0.001);
        let (low, high) = small.extent().unwrap();
        assert!((high - low).norm() < 0.02, "a metres-sized cloud should measure small");
    }

    #[test]
    fn a_pcd_with_packed_colour_reads() {
        let packed = f32::from_bits((255u32 << 16) | (128 << 8) | 64);
        let text = format!(
            "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\n\
             COUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3 {packed}\n"
        );
        let scene = read_pcd(text.as_bytes()).unwrap();
        assert_eq!(scene.points, vec![Point3::new(1.0, 2.0, 3.0)]);
        assert_eq!(scene.colors, vec![[255, 128, 64]]);
    }

    #[test]
    fn a_compressed_pcd_says_what_to_do_instead_of_failing_blankly() {
        let text = "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n\
                    WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary_compressed\n";
        let err = read_pcd(text.as_bytes()).unwrap_err();
        assert!(
            matches!(err, InterchangeError::Unsupported(ref why) if why.contains("PLY")),
            "{err}"
        );
    }

    #[test]
    fn an_obj_with_negative_and_slashed_indices_reads() {
        let text = "v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 1\nf -3//1 -2//1 -1//1\n";
        let scene = read_obj(text.as_bytes()).unwrap();
        assert_eq!(scene.indices, vec![0, 1, 2]);
        assert_eq!(scene.normals.len(), 3);
    }

    #[test]
    fn an_stl_welds_its_corners_into_shared_vertices() {
        // Two triangles sharing an edge: 6 corners, 4 vertices.
        let text = "solid t\n\
            facet normal 0 0 1\n outer loop\n  vertex 0 0 0\n  vertex 1 0 0\n  vertex 0 1 0\n endloop\nendfacet\n\
            facet normal 0 0 1\n outer loop\n  vertex 1 0 0\n  vertex 1 1 0\n  vertex 0 1 0\n endloop\nendfacet\n\
            endsolid t\n";
        let scene = read_stl_text(text.as_bytes()).unwrap();
        assert_eq!(scene.triangles(), 2);
        assert_eq!(scene.points.len(), 4, "the shared corners should be one vertex each");
    }
}
