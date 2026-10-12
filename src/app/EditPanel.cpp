#include "EditPanel.h"

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

namespace revoscan::app {

EditPanel::EditPanel(document::ScanDocument* document, QWidget* parent)
    : QWidget(parent), document_(document) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(6);

    auto* title = new QLabel(tr("Cloud Filters"), this);
    title->setObjectName("PanelTitle");
    root->addWidget(title);

    auto* hint = new QLabel(
        tr("Select points directly in the viewport with Rectangular / Lasso "
           "Selection, then Delete. Filters below apply to the whole cloud."),
        this);
    hint->setWordWrap(true);
    hint->setObjectName("StatusHint");
    root->addWidget(hint);

    root->addWidget(new QLabel(tr("Statistical outliers"), this));
    outlierNeighbors_ = new QSpinBox(this);
    outlierNeighbors_->setRange(4, 64);
    outlierNeighbors_->setValue(16);
    outlierStd_ = new QDoubleSpinBox(this);
    outlierStd_->setRange(0.1, 5.0);
    outlierStd_->setDecimals(2);
    outlierStd_->setSingleStep(0.1);
    outlierStd_->setValue(1.5);
    root->addWidget(outlierNeighbors_);
    root->addWidget(outlierStd_);
    auto* outlierBtn = new QPushButton(tr("Remove Outliers"), this);
    root->addWidget(outlierBtn);

    root->addWidget(new QLabel(tr("Base / table removal"), this));
    baseThresh_ = new QDoubleSpinBox(this);
    baseThresh_->setRange(0.001, 0.05);
    baseThresh_->setDecimals(3);
    baseThresh_->setSingleStep(0.001);
    baseThresh_->setValue(0.004);
    root->addWidget(baseThresh_);
    auto* baseBtn = new QPushButton(tr("Remove Base Plane"), this);
    root->addWidget(baseBtn);

    root->addWidget(new QLabel(tr("PCL Surface Smoothing"), this));
    auto* mlsBtn = new QPushButton(tr("MLS Smooth Surface (PCL)"), this);
    root->addWidget(mlsBtn);

    auto* clearBtn = new QPushButton(tr("Clear Document"), this);
    root->addWidget(clearBtn);
    root->addStretch(1);

    connect(outlierBtn, &QPushButton::clicked, this, [this] {
        if (!document_ || !document_->isMetric()) {
            emit editError(tr("Editing requires a metric scan document"));
            return;
        }
        if (!document_->removeStatisticalOutliers(
                static_cast<size_t>(outlierNeighbors_->value()), outlierStd_->value())) {
            emit editError(tr("Outlier removal failed"));
            return;
        }
        emit documentEdited();
    });
    connect(baseBtn, &QPushButton::clicked, this, [this] {
        if (!document_ || !document_->isMetric()) {
            emit editError(tr("Editing requires a metric scan document"));
            return;
        }
        if (!document_->basePlaneRemoval(baseThresh_->value())) {
            emit editError(tr("Base removal failed"));
            return;
        }
        emit documentEdited();
    });
    connect(mlsBtn, &QPushButton::clicked, this, [this] {
        if (!document_ || !document_->isMetric()) {
            emit editError(tr("Editing requires a metric scan document"));
            return;
        }
        if (!document_->smoothMLS(0.005)) {
            emit editError(tr("MLS surface smoothing failed"));
            return;
        }
        emit documentEdited();
    });
    connect(clearBtn, &QPushButton::clicked, this, [this] {
        if (document_) {
            document_->clear();
            emit documentEdited();
        }
    });
}

} // namespace revoscan::app
