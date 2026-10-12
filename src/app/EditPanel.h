#pragma once

#include <QWidget>

#include "document/ScanDocument.h"

class QDoubleSpinBox;
class QSpinBox;

namespace revoscan::app {

// Side panel with cloud filter tools. Viewport selection editing
// (rect/lasso/invert/delete) lives in PointCloudView's edit toolbar.
class EditPanel : public QWidget {
    Q_OBJECT
public:
    explicit EditPanel(document::ScanDocument* document, QWidget* parent = nullptr);

signals:
    void documentEdited();
    void editError(const QString& message);

private:
    document::ScanDocument* document_ = nullptr;
    QSpinBox* outlierNeighbors_ = nullptr;
    QDoubleSpinBox* outlierStd_ = nullptr;
    QDoubleSpinBox* baseThresh_ = nullptr;
};

} // namespace revoscan::app
