#include "PointCloudView.h"

#include <QDateTime>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QPainter>
#include <QPainterPath>
#include <QSurfaceFormat>
#include <QTimer>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace revoscan::app {
namespace {

constexpr const char* kVertexShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uMvp;
uniform float uExtent;
uniform float uPointScale;
out vec3 vColor;
out float vDepthTint;

void main() {
    gl_Position = uMvp * vec4(aPos, 1.0);
    // Vary point size based on depth: closer points are bigger.
    float z = (uMvp * vec4(aPos, 1.0)).z;
    float depthFactor = clamp(1.0 - z * 0.5, 0.4, 2.0);
    gl_PointSize = uPointScale * depthFactor;
    vColor = aColor;
    vDepthTint = clamp((aPos.z / max(uExtent, 1e-4)) + 0.5, 0.0, 1.0);
}
)";

constexpr const char* kFragmentShader = R"(
#version 330 core
uniform float uUseVertexColor;
in vec3 vColor;
in float vDepthTint;
out vec4 fragColor;

void main() {
    // Soft circular point (antialiased).
    vec2 circ = gl_PointCoord - vec2(0.5);
    float dist = length(circ);
    if (dist > 0.5) discard;
    float alpha = smoothstep(0.5, 0.35, dist);
    if (uUseVertexColor > 0.5) {
        fragColor = vec4(vColor, alpha);
    } else {
        float t = vDepthTint;
        vec3 col = vec3(0.16 + 0.16 * t, 0.47 + 0.35 * t, 0.78 + 0.16 * (1.0 - t));
        fragColor = vec4(col, alpha);
    }
}
)";

// Shaded mesh: position + normal + vertex color, lit by a camera headlight so
// the surface reads as a solid model (Meshed = clay, Textured = vertex colors).
constexpr const char* kMeshVertexShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
uniform mat4 uMvp;
uniform mat3 uNormalMat;
out vec3 vNormal;
out vec3 vColor;

void main() {
    gl_Position = uMvp * vec4(aPos, 1.0);
    vNormal = normalize(uNormalMat * aNormal);
    vColor = aColor;
}
)";

constexpr const char* kMeshFragmentShader = R"(
#version 330 core
in vec3 vNormal;
in vec3 vColor;
uniform float uUseVertexColor;
uniform vec3 uBaseColor;
uniform float uTime;
out vec4 fragColor;

void main() {
    vec3 n = normalize(vNormal);
    // Two-sided headlight from the camera (+Z) plus a little ambient fill.
    float diff = abs(n.z);
    float shade = 0.25 + 0.75 * diff;
    vec3 base = (uUseVertexColor > 0.5) ? vColor : uBaseColor;
    fragColor = vec4(base * shade, 1.0);
}
)";

// Revo-style selection highlight — vibrant green.
constexpr float kSelR = 0.32f;
constexpr float kSelG = 0.85f;
constexpr float kSelB = 0.35f;

// Clay material for the untextured "Meshed" stage.
constexpr float kClayR = 0.72f;
constexpr float kClayG = 0.72f;
constexpr float kClayB = 0.74f;

// Camera animation speed (degrees/frame).
constexpr float kAnimSpeed = 0.12f;

// Grid constants.
constexpr int kGridLinesPerAxis = 8;
constexpr float kGridMinExtent = 0.1f;
constexpr float kGridMaxExtent = 20.0f;

} // namespace

PointCloudView::PointCloudView(QWidget* parent) : QOpenGLWidget(parent) {
    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setSamples(4); // multi-sample anti-aliasing
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    setFormat(fmt);
    setMinimumSize(320, 240);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);

    animTimer_ = new QTimer(this);
    animTimer_->setInterval(16); // ~60 fps animation
    connect(animTimer_, &QTimer::timeout, this, &PointCloudView::animateCameraStep);
}

PointCloudView::~PointCloudView() {
    animTimer_->stop();
    if (context() && context()->isValid()) {
        makeCurrent();
        if (vao_) {
            vao_->destroy();
        }
        if (vbo_) {
            vbo_->destroy();
        }
        if (meshVao_) {
            meshVao_->destroy();
        }
        if (meshVbo_) {
            meshVbo_->destroy();
        }
        if (meshEbo_) {
            meshEbo_->destroy();
        }
        doneCurrent();
    }
    delete vao_;
    vao_ = nullptr;
    delete vbo_;
    vbo_ = nullptr;
    delete program_;
    program_ = nullptr;
    delete meshVao_;
    meshVao_ = nullptr;
    delete meshVbo_;
    meshVbo_ = nullptr;
    delete meshEbo_;
    meshEbo_ = nullptr;
    delete meshProgram_;
    meshProgram_ = nullptr;
}

void PointCloudView::setPointCloud(const pointcloud::PointCloud& cloud) {
    cloud_ = cloud;
    renderMode_ = RenderMode::Points;
    recomputeBounds();
    if (selection_.size() != cloud_.points.size()) {
        selection_.assign(cloud_.points.size(), 0);
        if (selectedCount_ != 0) {
            selectedCount_ = 0;
            emit selectionChanged(0);
        }
    }
    dirtyUpload_ = true;
    update();
}

void PointCloudView::setMesh(const geometry::TriangleMesh& mesh, bool textured) {
    renderMode_ = RenderMode::Mesh;
    meshTextured_ = textured && mesh.hasColors;

    cloud_.points.clear();
    pointCount_ = 0; // the point VBO is about to be emptied; never draw stale counts
    selection_.clear();
    if (selectedCount_ != 0) {
        selectedCount_ = 0;
        emit selectionChanged(0);
    }
    dirtyUpload_ = true;

    meshVertices_.clear();
    meshIndices_.clear();
    meshIndexCount_ = 0;

    const size_t vcount = mesh.vertices.size();
    if (vcount == 0 || mesh.indices.size() < 3) {
        dirtyMeshUpload_ = true;
        update();
        return;
    }

    meshVertices_.resize(vcount);
    for (size_t i = 0; i < vcount; ++i) {
        const auto& p = mesh.vertices[i];
        MeshVertex& v = meshVertices_[i];
        v.x = p.x;
        v.y = p.y;
        v.z = p.z;
        v.nx = v.ny = 0.0f;
        v.nz = 0.0f;
        v.r = p.r / 255.0f;
        v.g = p.g / 255.0f;
        v.b = p.b / 255.0f;
    }

    // Per-vertex normals as the area-weighted sum of adjacent face normals.
    // Triangles referencing missing vertices are dropped here so glDrawElements
    // can never read past the vertex buffer.
    meshIndices_.reserve(mesh.indices.size());
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const uint32_t ia = mesh.indices[t + 0];
        const uint32_t ib = mesh.indices[t + 1];
        const uint32_t ic = mesh.indices[t + 2];
        if (ia >= vcount || ib >= vcount || ic >= vcount) {
            continue;
        }
        meshIndices_.insert(meshIndices_.end(), {ia, ib, ic});
        const MeshVertex& a = meshVertices_[ia];
        const MeshVertex& b = meshVertices_[ib];
        const MeshVertex& c = meshVertices_[ic];
        const float ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
        const float vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
        const float nx = uy * vz - uz * vy;
        const float ny = uz * vx - ux * vz;
        const float nz = ux * vy - uy * vx;
        for (uint32_t idx : {ia, ib, ic}) {
            meshVertices_[idx].nx += nx;
            meshVertices_[idx].ny += ny;
            meshVertices_[idx].nz += nz;
        }
    }
    float minX = meshVertices_[0].x, maxX = minX;
    float minY = meshVertices_[0].y, maxY = minY;
    float minZ = meshVertices_[0].z, maxZ = minZ;
    for (MeshVertex& v : meshVertices_) {
        const float len =
            std::sqrt(v.nx * v.nx + v.ny * v.ny + v.nz * v.nz);
        if (len > 1e-8f) {
            v.nx /= len;
            v.ny /= len;
            v.nz /= len;
        } else {
            v.nx = 0.0f;
            v.ny = 0.0f;
            v.nz = 1.0f;
        }
        minX = std::min(minX, v.x);
        maxX = std::max(maxX, v.x);
        minY = std::min(minY, v.y);
        maxY = std::max(maxY, v.y);
        minZ = std::min(minZ, v.z);
        maxZ = std::max(maxZ, v.z);
    }
    meshIndexCount_ = meshIndices_.size();
    if (cloud_.points.empty()) {
        recomputeBoundsFrom(minX, minY, minZ, maxX, maxY, maxZ, vcount);
    }
    dirtyMeshUpload_ = true;
    update();
}

void PointCloudView::setColorEnabled(bool on) {
    colorEnabled_ = on;
    update();
}

void PointCloudView::setTool(Tool tool) {
    tool_ = tool;
    selecting_ = false;
    lasso_.clear();
    setCursor(tool_ == Tool::Orbit ? Qt::ArrowCursor : Qt::CrossCursor);
    update();
}

void PointCloudView::clearSelection() {
    setSelectionMask(std::vector<uint8_t>(cloud_.points.size(), 0));
}

void PointCloudView::invertSelection() {
    std::vector<uint8_t> mask(cloud_.points.size(), 0);
    for (size_t i = 0; i < selection_.size() && i < mask.size(); ++i) {
        mask[i] = selection_[i] ? 0 : 1;
    }
    setSelectionMask(std::move(mask));
}

void PointCloudView::autoFitView() {
    if (pointCount_ == 0 && meshIndexCount_ == 0) {
        return;
    }
    targetZoom_ = 1.0f;
    targetYaw_ = yawDeg_;
    targetPitch_ = 30.0f; // slight elevated default
    if (!animTimer_->isActive()) {
        animTimer_->start();
        animating_ = true;
    }
    emit viewModeChanged();
}

void PointCloudView::resetView() {
    targetZoom_ = 1.0f;
    targetYaw_ = 45.0f;
    targetPitch_ = 30.0f;
    if (!animTimer_->isActive()) {
        animTimer_->start();
        animating_ = true;
    }
    emit viewModeChanged();
}

void PointCloudView::setGroundGridVisible(bool visible) {
    showGroundGrid_ = visible;
    update();
}

std::vector<size_t> PointCloudView::selectedIndices() const {
    std::vector<size_t> out;
    out.reserve(selectedCount_);
    for (size_t i = 0; i < selection_.size(); ++i) {
        if (selection_[i]) {
            out.push_back(i);
        }
    }
    return out;
}

void PointCloudView::setSelectionMask(std::vector<uint8_t> mask) {
    selection_ = std::move(mask);
    selection_.resize(cloud_.points.size(), 0);
    selectedCount_ = static_cast<size_t>(
        std::count(selection_.begin(), selection_.end(), uint8_t{1}));
    dirtyUpload_ = true;
    emit selectionChanged(static_cast<int>(selectedCount_));
    update();
}

void PointCloudView::recomputeBounds() {
    if (cloud_.points.empty()) {
        center_ = QVector3D(0.0f, 0.0f, 0.0f);
        extent_ = 1.0f;
        pointCount_ = 0;
        return;
    }

    float minX = cloud_.points[0].x, maxX = minX;
    float minY = cloud_.points[0].y, maxY = minY;
    float minZ = cloud_.points[0].z, maxZ = minZ;
    for (const auto& p : cloud_.points) {
        minX = std::min(minX, p.x);
        maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
        minZ = std::min(minZ, p.z);
        maxZ = std::max(maxZ, p.z);
    }
    center_ = QVector3D((minX + maxX) * 0.5f,
                        (minY + maxY) * 0.5f,
                        (minZ + maxZ) * 0.5f);
    extent_ = std::max({maxX - minX, maxY - minY, maxZ - minZ, 1e-4f});
    pointCount_ = cloud_.points.size();
}

void PointCloudView::recomputeBoundsFrom(float minX, float minY, float minZ,
                                         float maxX, float maxY, float maxZ,
                                         size_t count) {
    if (count == 0) {
        center_ = QVector3D(0.0f, 0.0f, 0.0f);
        extent_ = 1.0f;
        return;
    }
    center_ = QVector3D((minX + maxX) * 0.5f,
                        (minY + maxY) * 0.5f,
                        (minZ + maxZ) * 0.5f);
    extent_ = std::max({maxX - minX, maxY - minY, maxZ - minZ, 1e-4f});
}

void PointCloudView::uploadMesh() {
    dirtyMeshUpload_ = false;
    if (!meshVao_ || !meshVbo_ || !meshEbo_) {
        return;
    }
    meshVao_->bind();
    meshVbo_->bind();
    if (meshVertices_.empty()) {
        meshVbo_->allocate(nullptr, 0);
    } else {
        meshVbo_->allocate(
            meshVertices_.data(),
            static_cast<int>(meshVertices_.size() * sizeof(MeshVertex)));
    }
    const int stride = static_cast<int>(sizeof(MeshVertex));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<const void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<const void*>(6 * sizeof(float)));
    meshEbo_->bind();
    if (meshIndices_.empty()) {
        meshEbo_->allocate(nullptr, 0);
    } else {
        meshEbo_->allocate(
            meshIndices_.data(),
            static_cast<int>(meshIndices_.size() * sizeof(uint32_t)));
    }
    meshVao_->release();
    meshVbo_->release();
    meshEbo_->release();
}

void PointCloudView::uploadCloud() {
    vertices_.clear();
    vertices_.reserve(cloud_.points.size());
    for (size_t i = 0; i < cloud_.points.size(); ++i) {
        const auto& p = cloud_.points[i];
        GpuVertex v;
        v.x = p.x;
        v.y = p.y;
        v.z = p.z;
        if (i < selection_.size() && selection_[i]) {
            v.r = kSelR;
            v.g = kSelG;
            v.b = kSelB;
        } else {
            v.r = p.r / 255.0f;
            v.g = p.g / 255.0f;
            v.b = p.b / 255.0f;
        }
        vertices_.push_back(v);
    }

    if (!vbo_ || !vao_) {
        pointCount_ = 0;
        return;
    }
    // Draw exactly what the buffer holds, whatever callers did to cloud_.
    pointCount_ = vertices_.size();
    vao_->bind();
    vbo_->bind();
    if (vertices_.empty()) {
        vbo_->allocate(nullptr, 0);
    } else {
        vbo_->allocate(vertices_.data(),
                       static_cast<int>(vertices_.size() * sizeof(GpuVertex)));
    }
    const int stride = static_cast<int>(sizeof(GpuVertex));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1, 3, GL_FLOAT, GL_FALSE, stride,
        reinterpret_cast<const void*>(3 * sizeof(float)));
    vbo_->release();
    vao_->release();
    dirtyUpload_ = false;
}

void PointCloudView::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    program_ = new QOpenGLShaderProgram();
    if (!program_->addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShader) ||
        !program_->addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader) ||
        !program_->link()) {
    }

    vao_ = new QOpenGLVertexArrayObject();
    vao_->create();
    vbo_ = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    vbo_->create();
    vbo_->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    dirtyUpload_ = true;

    meshProgram_ = new QOpenGLShaderProgram();
    if (!meshProgram_->addShaderFromSourceCode(QOpenGLShader::Vertex,
                                               kMeshVertexShader) ||
        !meshProgram_->addShaderFromSourceCode(QOpenGLShader::Fragment,
                                               kMeshFragmentShader) ||
        !meshProgram_->link()) {
    }
    meshVao_ = new QOpenGLVertexArrayObject();
    meshVao_->create();
    meshVbo_ = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    meshVbo_->create();
    meshVbo_->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    meshEbo_ = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
    meshEbo_->create();
    meshEbo_->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    dirtyMeshUpload_ = true;
}

void PointCloudView::resizeGL(int w, int h) {
    viewportW_ = std::max(w, 1);
    viewportH_ = std::max(h, 1);
    glViewport(0, 0, viewportW_, viewportH_);
}

QMatrix4x4 PointCloudView::mvpMatrix() const {
    QMatrix4x4 mvp;
    const float aspect =
        static_cast<float>(viewportW_) / static_cast<float>(viewportH_);
    float half = extent_ * 0.6f / zoomFactor_;
    float halfX = half;
    float halfY = half;
    if (aspect >= 1.0f) {
        halfX = half * aspect;
    } else {
        halfY = half / aspect;
    }
    mvp.ortho(-halfX, halfX, -halfY, halfY, -extent_ * 4.0f, extent_ * 4.0f);
    mvp.rotate(pitchDeg_, 1.0f, 0.0f, 0.0f);
    mvp.rotate(yawDeg_, 0.0f, 1.0f, 0.0f);
    mvp.translate(-center_);
    return mvp;
}

QMatrix4x4 PointCloudView::rotationMatrix() const {
    QMatrix4x4 r;
    r.rotate(pitchDeg_, 1.0f, 0.0f, 0.0f);
    r.rotate(yawDeg_, 0.0f, 1.0f, 0.0f);
    return r;
}

QPointF PointCloudView::projectToWidget(const pointcloud::Point& p,
                                        const QMatrix4x4& mvp) const {
    const QVector3D ndc = mvp.map(QVector3D(p.x, p.y, p.z));
    const float sx = (ndc.x() * 0.5f + 0.5f) * static_cast<float>(width());
    const float sy = (1.0f - (ndc.y() * 0.5f + 0.5f)) * static_cast<float>(height());
    return QPointF(sx, sy);
}

void PointCloudView::applySelectionShape(bool additive) {
    if (cloud_.points.empty()) {
        return;
    }

    QPainterPath path;
    if (tool_ == Tool::RectSelect) {
        path.addRect(QRectF(selectStart_, selectCurrent_).normalized());
    } else if (tool_ == Tool::LassoSelect && lasso_.size() >= 3) {
        path.addPolygon(lasso_);
        path.closeSubpath();
    } else {
        return;
    }

    std::vector<uint8_t> mask =
        additive ? selection_ : std::vector<uint8_t>(cloud_.points.size(), 0);
    mask.resize(cloud_.points.size(), 0);

    const QMatrix4x4 mvp = mvpMatrix();
    for (size_t i = 0; i < cloud_.points.size(); ++i) {
        if (path.contains(projectToWidget(cloud_.points[i], mvp))) {
            mask[i] = 1;
        }
    }
    setSelectionMask(std::move(mask));
}

void PointCloudView::drawGroundGrid(QPainter& painter, const QMatrix4x4& mvp) {
    if (!showGroundGrid_ || extent_ <= 0.0f) {
        return;
    }

    // Determine grid size from extent, capped.
    const float gridExt = std::clamp(extent_ * 1.2f, kGridMinExtent, kGridMaxExtent);
    const float halfGrid = gridExt * 0.5f;
    const int lines = kGridLinesPerAxis;

    // Compute grid origin to be roughly at z=0 or at the minimum z of the scene.
    const float gridZ = center_.z() - extent_ * 0.55f;

    QPen majorPen(QColor(0x3a, 0x3f, 0x45, 120), 1);
    QPen minorPen(QColor(0x3a, 0x3f, 0x45, 60), 1);

    // Project the grid corners to determine screen-space bounds — skip if
    // entirely behind camera.
    const QVector3D gridCenter(center_.x(), center_.y(), gridZ);
    const QVector3D projected = mvp.map(gridCenter);
    // Cull if the grid center is behind the near plane.
    if (projected.z() > 1.0f || projected.z() < -1.0f) {
        // If the grid is behind the view, skip rendering it.
        // We check whether the entire grid is behind the camera.
        bool allBehind = true;
        for (int gx = -1; gx <= 1; gx += 2) {
            for (int gy = -1; gy <= 1; gy += 2) {
                const QVector3D corner(center_.x() + gx * halfGrid,
                                        center_.y() + gy * halfGrid,
                                        gridZ);
                const QVector3D cp = mvp.map(corner);
                if (cp.z() <= 1.0f && cp.z() >= -1.0f) {
                    allBehind = false;
                }
            }
        }
        if (allBehind) return;
    }

    painter.setRenderHint(QPainter::Antialiasing, true);

    // Draw a subtle ground-plane fill.
    QPointF gridPts[4];
    for (int i = 0; i < 4; ++i) {
        const int gx = (i == 0 || i == 3) ? -1 : 1;
        const int gy = (i < 2) ? -1 : 1;
        const QVector3D corner(center_.x() + gx * halfGrid,
                                center_.y() + gy * halfGrid,
                                gridZ);
        const QVector3D ndc = mvp.map(corner);
        const float sx = (ndc.x() * 0.5f + 0.5f) * static_cast<float>(width());
        const float sy = (1.0f - (ndc.y() * 0.5f + 0.5f)) * static_cast<float>(height());
        gridPts[i] = QPointF(sx, sy);
    }
    QPainterPath fillPath;
    {
        QPolygonF poly;
        poly << gridPts[0] << gridPts[1] << gridPts[2] << gridPts[3];
        fillPath.addPolygon(poly);
    }
    painter.fillPath(fillPath, QColor(0x1b, 0x1d, 0x21, 120));

    // Draw grid lines.
    for (int i = 0; i <= lines; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(lines);
        const float pos = -halfGrid + t * gridExt;
        const bool isMajor = (i % (lines / 2)) == 0;

        // X-aligned line.
        const QVector3D startX(center_.x() - halfGrid, center_.y() + pos, gridZ);
        const QVector3D endX(center_.x() + halfGrid, center_.y() + pos, gridZ);
        const QVector3D ndcSx = mvp.map(startX);
        const QVector3D ndcEx = mvp.map(endX);
        const QPointF sSx((ndcSx.x() * 0.5f + 0.5f) * width(),
                           (1.0f - (ndcSx.y() * 0.5f + 0.5f)) * height());
        const QPointF sEx((ndcEx.x() * 0.5f + 0.5f) * width(),
                           (1.0f - (ndcEx.y() * 0.5f + 0.5f)) * height());
        painter.setPen(isMajor ? majorPen : minorPen);
        painter.drawLine(sSx, sEx);

        // Y-aligned line.
        const QVector3D startY(center_.x() + pos, center_.y() - halfGrid, gridZ);
        const QVector3D endY(center_.x() + pos, center_.y() + halfGrid, gridZ);
        const QVector3D ndcSy = mvp.map(startY);
        const QVector3D ndcEy = mvp.map(endY);
        const QPointF sSy((ndcSy.x() * 0.5f + 0.5f) * width(),
                           (1.0f - (ndcSy.y() * 0.5f + 0.5f)) * height());
        const QPointF sEy((ndcEy.x() * 0.5f + 0.5f) * width(),
                           (1.0f - (ndcEy.y() * 0.5f + 0.5f)) * height());
        painter.drawLine(sSy, sEy);
    }
}

void PointCloudView::drawAxisIndicator(QPainter& painter) {
    // Draw a miniature XYZ axis indicator in the bottom-left corner.
    const int margin = 12;
    const int axisLen = 36;
    const int cx = margin + axisLen;
    const int cy = height() - margin - axisLen;

    // Build a simple view-rotation-only matrix.
    QMatrix4x4 rot;
    rot.rotate(pitchDeg_, 1.0f, 0.0f, 0.0f);
    rot.rotate(yawDeg_, 0.0f, 1.0f, 0.0f);

    struct Axis { QColor color; QVector3D dir; QString label; };
    const Axis axes[] = {
        {QColor(0xd9, 0x5c, 0x5c), QVector3D(1, 0, 0), "X"},
        {QColor(0x5b, 0xbf, 0x5b), QVector3D(0, 1, 0), "Y"},
        {QColor(0x5b, 0x9f, 0xef), QVector3D(0, 0, 1), "Z"},
    };

    painter.setRenderHint(QPainter::Antialiasing, true);

    for (const auto& axis : axes) {
        const QVector3D end = rot.map(axis.dir * axisLen);
        const int ex = cx + static_cast<int>(end.x());
        const int ey = cy - static_cast<int>(end.y());

        QPen pen(axis.color, 2);
        painter.setPen(pen);
        painter.drawLine(cx, cy, ex, ey);

        // Draw label at the endpoint.
        painter.setPen(axis.color);
        QFont f = painter.font();
        f.setPointSize(9);
        f.setBold(true);
        painter.setFont(f);
        painter.drawText(ex + 2, ey + 4, axis.label);
    }

    // Draw a small circle at the origin.
    painter.setPen(QPen(QColor(0x8b, 0x91, 0x98, 100), 1));
    painter.setBrush(QColor(0x8b, 0x91, 0x98, 40));
    painter.drawEllipse(QPointF(cx, cy), 3, 3);
}

void PointCloudView::animateCameraStep() {
    // Smoothly interpolate towards target values.
    const float kEpsilon = 0.5f;

    float dyaw = targetYaw_ - yawDeg_;
    float dpitch = targetPitch_ - pitchDeg_;
    float dzoom = targetZoom_ - zoomFactor_;

    if (std::fabs(dyaw) < kEpsilon && std::fabs(dpitch) < kEpsilon &&
        std::fabs(dzoom) < 0.01f) {
        yawDeg_ = targetYaw_;
        pitchDeg_ = targetPitch_;
        zoomFactor_ = targetZoom_;
        animTimer_->stop();
        animating_ = false;
        update();
        return;
    }

    yawDeg_ += dyaw * kAnimSpeed;
    pitchDeg_ += dpitch * kAnimSpeed;
    zoomFactor_ += dzoom * kAnimSpeed;
    update();
}

void PointCloudView::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (dirtyUpload_) {
        uploadCloud();
    }
    if (dirtyMeshUpload_) {
        uploadMesh();
    }

    // Draw the 3D content (mesh or points) first.
    if (renderMode_ == RenderMode::Mesh && meshProgram_ &&
        meshProgram_->isLinked() && meshIndexCount_ > 0 && meshVao_) {
        meshProgram_->bind();
        meshVao_->bind();
        meshProgram_->setUniformValue("uMvp", mvpMatrix());
        meshProgram_->setUniformValue("uNormalMat", rotationMatrix().normalMatrix());
        meshProgram_->setUniformValue("uUseVertexColor", meshTextured_ ? 1.0f : 0.0f);
        meshProgram_->setUniformValue("uBaseColor",
                                      QVector3D(kClayR, kClayG, kClayB));
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(meshIndexCount_),
                       GL_UNSIGNED_INT, nullptr);
        meshVao_->release();
        meshProgram_->release();
    }

    if (program_ && program_->isLinked() && pointCount_ > 0 && vao_) {
        program_->bind();
        vao_->bind();
        program_->setUniformValue("uMvp", mvpMatrix());
        const bool useVertexColor = colorEnabled_ || selectedCount_ > 0;
        program_->setUniformValue("uUseVertexColor", useVertexColor ? 1.0f : 0.0f);
        program_->setUniformValue("uExtent", extent_);
        program_->setUniformValue("uPointScale", std::clamp(3.0f / zoomFactor_, 1.0f, 8.0f));
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(pointCount_));
        vao_->release();
        program_->release();
    }

    // Overlay: QPainter for ground grid, axis indicator, selection, HUD.
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    // Draw ground grid before selection overlay.
    drawGroundGrid(painter, mvpMatrix());

    // Coordinate axis indicator.
    drawAxisIndicator(painter);

    // Live selection shape overlay with animated dashes.
    if (selecting_) {
        if (selAnimStartMs_ == 0) {
            selAnimStartMs_ = QDateTime::currentMSecsSinceEpoch();
        }
        const qint64 elapsed = QDateTime::currentMSecsSinceEpoch() - selAnimStartMs_;
        const float phase = static_cast<float>(elapsed % 800) / 800.0f; // 800ms cycle
        const int dashPhase = static_cast<int>(phase * 16.0f);

        QPen selPen(QColor(0x2b, 0xd9, 0x9f), 1.5);
        QVector<qreal> dashes;
        dashes << 6.0 << 4.0;
        selPen.setDashPattern(dashes);
        selPen.setDashOffset(static_cast<qreal>(dashPhase));
        painter.setPen(selPen);
        painter.setBrush(QColor(43, 217, 159, 25));

        if (tool_ == Tool::RectSelect) {
            painter.drawRect(QRectF(selectStart_, selectCurrent_).normalized());
        } else if (tool_ == Tool::LassoSelect && lasso_.size() >= 2) {
            painter.drawPolyline(lasso_);
        }
    } else {
        selAnimStartMs_ = 0;
    }

    // HUD text.
    painter.setPen(QColor(0x5b, 0x61, 0x69));

    // Viewport mode indicator (top-left).
    if (renderMode_ == RenderMode::Mesh) {
        painter.drawText(
            QRect(8, 8, width() - 16, 20),
            Qt::AlignLeft | Qt::AlignVCenter,
            tr("%1 triangles%2")
                .arg(static_cast<qulonglong>(meshIndexCount_ / 3))
                .arg(meshTextured_ ? tr(" — textured") : tr(" — shaded")));
    }

    // Bottom-left info text.
    if (pointCount_ == 0 && meshIndexCount_ == 0) {
        painter.setPen(QColor(0x8b, 0x91, 0x98));
        QFont hintFont = painter.font();
        hintFont.setPointSize(11);
        painter.setFont(hintFont);
        painter.drawText(rect(), Qt::AlignCenter,
                         tr("Connect a scanner and press Scan\n"
                            "Or use File > Open PLY… to load an existing scan"));
    } else {
        QFont infoFont = painter.font();
        infoFont.setPointSize(10);
        painter.setFont(infoFont);
        const QString info = renderMode_ == RenderMode::Points
            ? (selectedCount_ > 0
                ? tr("%1 pts  •  %2 selected  •  F: Fit  R: Reset")
                    .arg(static_cast<qulonglong>(pointCount_))
                    .arg(static_cast<qulonglong>(selectedCount_))
                : tr("%1 pts  •  F: Fit  R: Reset")
                    .arg(static_cast<qulonglong>(pointCount_)))
            : tr("F: Fit  R: Reset");
        painter.drawText(QRect(8, height() - 22, width() - 16, 20),
                         Qt::AlignLeft | Qt::AlignVCenter, info);
    }
}

void PointCloudView::mousePressEvent(QMouseEvent* event) {
    setFocus();
    if (tool_ == Tool::Orbit) {
        dragging_ = true;
        lastMousePos_ = event->pos();
        return;
    }
    selecting_ = true;
    selectStart_ = event->pos();
    selectCurrent_ = event->pos();
    lasso_.clear();
    lasso_ << QPointF(event->pos());
    update();
}

void PointCloudView::mouseMoveEvent(QMouseEvent* event) {
    if (tool_ == Tool::Orbit) {
        if (!dragging_) {
            return;
        }
        const QPoint delta = event->pos() - lastMousePos_;
        lastMousePos_ = event->pos();
        yawDeg_ += delta.x() * 0.5f;
        pitchDeg_ += delta.y() * 0.5f;
        update();
        return;
    }
    if (!selecting_) {
        return;
    }
    selectCurrent_ = event->pos();
    if (tool_ == Tool::LassoSelect) {
        lasso_ << QPointF(event->pos());
    }
    update();
}

void PointCloudView::mouseReleaseEvent(QMouseEvent* event) {
    if (tool_ == Tool::Orbit) {
        dragging_ = false;
        return;
    }
    if (!selecting_) {
        return;
    }
    selecting_ = false;
    selectCurrent_ = event->pos();
    applySelectionShape(event->modifiers().testFlag(Qt::ShiftModifier));
    lasso_.clear();
    update();
}

void PointCloudView::wheelEvent(QWheelEvent* event) {
    zoomFactor_ *= (event->angleDelta().y() > 0) ? 1.1f : 0.9f;
    zoomFactor_ = std::clamp(zoomFactor_, 0.1f, 20.0f);
    update();
}

void PointCloudView::keyPressEvent(QKeyEvent* event) {
    switch (event->key()) {
    case Qt::Key_F:
        autoFitView();
        break;
    case Qt::Key_R:
        resetView();
        break;
    case Qt::Key_Escape:
        if (selectedCount_ > 0) {
            clearSelection();
        }
        break;
    case Qt::Key_Delete:
    case Qt::Key_Backspace:
        // Selection delete is handled by MainWindow via the delete button.
        break;
    default:
        QOpenGLWidget::keyPressEvent(event);
    }
}

} // namespace revoscan::app
