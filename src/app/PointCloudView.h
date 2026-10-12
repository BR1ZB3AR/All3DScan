#pragma once

#include <QElapsedTimer>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QMatrix4x4>
#include <QPolygonF>
#include <QVector3D>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry/TriangleMesh.h"
#include "pointcloud/PointCloud.h"

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;
class QTimer;

namespace revoscan::app {

// GPU point-cloud viewport with Revo Scan-style viewport editing:
// orbit / rectangular selection / lasso selection, selection highlight,
// invert, and delete (delete is applied by MainWindow via ScanDocument).
// Also renders a shaded/textured triangle mesh for the Model stage tabs.
// Includes ground-grid overlay, coordinate axis indicator, and auto-fit.
class PointCloudView : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    enum class Tool {
        Orbit,
        RectSelect,
        LassoSelect,
    };

    enum class RenderMode {
        Points,
        Mesh,
    };

    explicit PointCloudView(QWidget* parent = nullptr);
    ~PointCloudView() override;

    Tool tool() const { return tool_; }
    RenderMode renderMode() const { return renderMode_; }
    size_t selectedCount() const { return selectedCount_; }
    std::vector<size_t> selectedIndices() const;
    bool groundGridVisible() const { return showGroundGrid_; }

public slots:
    void setPointCloud(const pointcloud::PointCloud& cloud);
    // Render a triangle mesh (shaded clay when textured=false, vertex colors
    // when textured=true). Switches the viewport into Mesh render mode.
    void setMesh(const geometry::TriangleMesh& mesh, bool textured);
    void setColorEnabled(bool on);
    void setTool(Tool tool);
    void clearSelection();
    void invertSelection();
    // Smoothly re-centre the view so the entire scene fits with padding.
    void autoFitView();
    // Reset to a default elevated perspective.
    void resetView();
    void setGroundGridVisible(bool visible);

signals:
    void selectionChanged(int count);
    void viewModeChanged();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    struct GpuVertex {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float r = 0.0f;
        float g = 0.0f;
        float b = 0.0f;
    };

    struct MeshVertex {
        float x = 0.0f, y = 0.0f, z = 0.0f;
        float nx = 0.0f, ny = 0.0f, nz = 1.0f;
        float r = 0.0f, g = 0.0f, b = 0.0f;
    };

    void uploadCloud();
    void uploadMesh();
    void recomputeBounds();
    void recomputeBoundsFrom(float minX, float minY, float minZ,
                             float maxX, float maxY, float maxZ, size_t count);
    QMatrix4x4 mvpMatrix() const;
    QMatrix4x4 rotationMatrix() const;
    QPointF projectToWidget(const pointcloud::Point& p, const QMatrix4x4& mvp) const;
    void applySelectionShape(bool additive);
    void setSelectionMask(std::vector<uint8_t> mask);
    void drawGroundGrid(QPainter& painter, const QMatrix4x4& mvp);
    void drawAxisIndicator(QPainter& painter);
    void animateCameraStep();

    pointcloud::PointCloud cloud_;
    std::vector<GpuVertex> vertices_;
    std::size_t pointCount_ = 0;
    bool dirtyUpload_ = false;

    RenderMode renderMode_ = RenderMode::Points;
    std::vector<MeshVertex> meshVertices_;
    std::vector<uint32_t> meshIndices_;
    std::size_t meshIndexCount_ = 0;
    bool meshTextured_ = false;
    bool dirtyMeshUpload_ = false;

    QVector3D center_{0.0f, 0.0f, 0.0f};
    float extent_ = 1.0f;

    // Animated camera target values.
    float targetYaw_ = 0.0f;
    float targetPitch_ = 0.0f;
    float targetZoom_ = 1.0f;
    float yawDeg_ = 0.0f;
    float pitchDeg_ = 30.0f; // slight elevation by default
    float zoomFactor_ = 1.0f;
    QPoint lastMousePos_;
    bool dragging_ = false;
    bool animating_ = false;
    QTimer* animTimer_ = nullptr;

    bool colorEnabled_ = true;
    bool showGroundGrid_ = true;

    Tool tool_ = Tool::Orbit;
    bool selecting_ = false;
    QPoint selectStart_;
    QPoint selectCurrent_;
    QPolygonF lasso_;
    qint64 selAnimStartMs_ = 0;
    std::vector<uint8_t> selection_; // per-point flag, size == cloud_.points.size()
    size_t selectedCount_ = 0;

    QOpenGLShaderProgram* program_ = nullptr;
    QOpenGLBuffer* vbo_ = nullptr;
    QOpenGLVertexArrayObject* vao_ = nullptr;

    QOpenGLShaderProgram* meshProgram_ = nullptr;
    QOpenGLBuffer* meshVbo_ = nullptr;
    QOpenGLBuffer* meshEbo_ = nullptr;
    QOpenGLVertexArrayObject* meshVao_ = nullptr;

    int viewportW_ = 1;
    int viewportH_ = 1;
};

} // namespace revoscan::app
