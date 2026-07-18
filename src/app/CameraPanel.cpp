#include "CameraPanel.h"

#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QSpinBox>
#include <QToolButton>
#include <QVBoxLayout>

namespace revoscan::app {
namespace {

QWidget* makeFieldRow(const QString& label, bool withAuto, QWidget* parent,
                      QToolButton** autoBtnOut = nullptr) {
    auto* row = new QWidget(parent);
    auto* h = new QHBoxLayout(row);
    h->setContentsMargins(8, 2, 8, 2);
    h->setSpacing(6);

    auto* name = new QLabel(label, row);
    name->setObjectName("FieldLabel");
    name->setMinimumWidth(64);
    h->addWidget(name);
    h->addStretch(1);

    if (withAuto) {
        auto* autoBtn = new QToolButton(row);
        autoBtn->setObjectName("CamAutoBtn");
        autoBtn->setText(QObject::tr("Auto"));
        autoBtn->setCheckable(true);
        autoBtn->setChecked(true);
        autoBtn->setFixedWidth(44);
        h->addWidget(autoBtn);
        if (autoBtnOut) {
            *autoBtnOut = autoBtn;
        }
    }

    auto* dec = new QToolButton(row);
    dec->setObjectName("CamStepBtn");
    dec->setText(QStringLiteral("\u2039"));
    dec->setFixedWidth(20);
    auto* spin = new QSpinBox(row);
    spin->setObjectName("CamSpin");
    spin->setRange(0, 100);
    spin->setValue(1);
    spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    spin->setFixedWidth(40);
    spin->setAlignment(Qt::AlignCenter);
    auto* inc = new QToolButton(row);
    inc->setObjectName("CamStepBtn");
    inc->setText(QStringLiteral("\u203a"));
    inc->setFixedWidth(20);

    QObject::connect(dec, &QToolButton::clicked, spin, [spin] { spin->stepDown(); });
    QObject::connect(inc, &QToolButton::clicked, spin, [spin] { spin->stepUp(); });

    h->addWidget(dec);
    h->addWidget(spin);
    h->addWidget(inc);
    return row;
}

} // namespace

CameraPanel::CameraPanel(const QString& title, Kind kind, QWidget* parent)
    : QWidget(parent), kind_(kind) {
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(0);

    auto* titleLabel = new QLabel(title, this);
    titleLabel->setObjectName("PanelTitle");
    v->addWidget(titleLabel);

    v->addWidget(makeFieldRow(tr("Exposure"), true, this, &exposureAuto_));
    if (kind_ == Kind::Rgb) {
        v->addWidget(makeFieldRow(tr("AWB"), true, this));
    }

    preview_ = new QLabel(this);
    preview_->setObjectName("PreviewArea");
    preview_->setAlignment(Qt::AlignCenter);
    preview_->setMinimumHeight(150);
    preview_->setText(tr("No signal"));
    preview_->setScaledContents(false);
    v->addWidget(preview_, 1);
}

void CameraPanel::setPreview(const QImage& image) {
    if (image.isNull()) {
        clearPreview();
        return;
    }
    preview_->setPixmap(QPixmap::fromImage(image).scaled(
        preview_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void CameraPanel::clearPreview() {
    preview_->setText(tr("No signal"));
}

} // namespace revoscan::app
