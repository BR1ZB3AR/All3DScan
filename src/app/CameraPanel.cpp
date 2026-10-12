#include "CameraPanel.h"

#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QPainter>
#include <QPixmap>
#include <QSpinBox>
#include <QTimer>
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
    showDepthMap_ = (kind_ == Kind::Depth);

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

    // Setup overlay layout on the preview label
    auto* overlayLayout = new QGridLayout(preview_);
    overlayLayout->setContentsMargins(8, 8, 8, 8);

    if (kind_ == Kind::Depth) {
        modeToggleBtn_ = new QToolButton(preview_);
        modeToggleBtn_->setObjectName("OverlayModeBtn");
        modeToggleBtn_->setText(showDepthMap_ ? tr("Depth") : tr("IR"));
        modeToggleBtn_->setCheckable(true);
        modeToggleBtn_->setChecked(showDepthMap_);
        overlayLayout->addWidget(modeToggleBtn_, 0, 1, Qt::AlignTop | Qt::AlignRight);
        connect(modeToggleBtn_, &QToolButton::clicked, this, &CameraPanel::onModeToggle);

        zoomGroupWidget_ = new QWidget(preview_);
        zoomGroupWidget_->setObjectName("ZoomGroup");
        auto* zh = new QHBoxLayout(zoomGroupWidget_);
        zh->setContentsMargins(0, 0, 0, 0);
        zh->setSpacing(2);

        const QString levels[] = {QStringLiteral("1x"), QStringLiteral("1.5x"), QStringLiteral("2x")};
        for (const auto& lvl : levels) {
            auto* btn = new QToolButton(zoomGroupWidget_);
            btn->setObjectName("OverlayZoomBtn");
            btn->setText(lvl);
            btn->setCheckable(true);
            btn->setChecked(lvl == QStringLiteral("1x"));
            zh->addWidget(btn);
            connect(btn, &QToolButton::clicked, this, &CameraPanel::onZoomSelect);
        }
        overlayLayout->addWidget(zoomGroupWidget_, 1, 0, 1, 2, Qt::AlignBottom | Qt::AlignCenter);
    }

    v->addWidget(preview_, 1);
}

void CameraPanel::setPreview(const QImage& image) {
    if (image.isNull()) {
        clearPreview();
        return;
    }
    lastImage_ = image;
    preview_->setText(QString());

    QImage cropped = image;
    if (zoomFactor_ > 1.0f) {
        int cropW = static_cast<int>(image.width() / zoomFactor_);
        int cropH = static_cast<int>(image.height() / zoomFactor_);
        int cropX = (image.width() - cropW) / 2;
        int cropY = (image.height() - cropH) / 2;
        cropped = image.copy(cropX, cropY, cropW, cropH);
    }

    const QSize area = preview_->contentsRect().size();
    if (area.width() < 16 || area.height() < 16) {
        preview_->setPixmap(QPixmap::fromImage(cropped));
        return;
    }
    QPixmap scaled = QPixmap::fromImage(cropped).scaled(
        area, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    preview_->setPixmap(scaled);
}

void CameraPanel::clearPreview() {
    preview_->setText(tr("No signal"));
    lastImage_ = QImage();
    // Draw a subtle "No signal" indicator.
    if (preview_->width() > 32 && preview_->height() > 32) {
        QPixmap empty(preview_->size());
        empty.fill(QColor(0x10, 0x12, 0x14));
        QPainter p(&empty);
        p.setRenderHint(QPainter::Antialiasing, true);

        // Subtle grid pattern.
        p.setPen(QColor(0x2b, 0x2f, 0x33, 80));
        for (int x = 0; x < empty.width(); x += 20) {
            p.drawLine(x, 0, x, empty.height());
        }
        for (int y = 0; y < empty.height(); y += 20) {
            p.drawLine(0, y, empty.width(), y);
        }

        // Icon hint (camera silhouette-like).
        p.setPen(QColor(0x5b, 0x61, 0x69, 100));
        const int iconSize = 32;
        const int iconX = (empty.width() - iconSize) / 2;
        const int iconY = (empty.height() - iconSize) / 2 - 12;
        p.drawRoundedRect(iconX, iconY, iconSize, iconSize * 0.7, 4, 4);
        p.drawEllipse(QPoint(empty.width() / 2, iconY + iconSize / 3), 7, 5);

        p.setPen(QColor(0x5b, 0x61, 0x69));
        QFont f = p.font();
        f.setPointSize(9);
        p.setFont(f);
        p.drawText(QRect(0, iconY + iconSize + 4, empty.width(), 20),
                   Qt::AlignCenter, tr("No signal"));

        p.end();
        preview_->setPixmap(empty);
    }
}

void CameraPanel::onModeToggle() {
    showDepthMap_ = modeToggleBtn_->isChecked();
    modeToggleBtn_->setText(showDepthMap_ ? tr("Depth") : tr("IR"));
    emit depthModeToggled(showDepthMap_);
}

void CameraPanel::onZoomSelect() {
    auto* senderBtn = qobject_cast<QToolButton*>(sender());
    if (!senderBtn || !zoomGroupWidget_) return;

    const auto buttons = zoomGroupWidget_->findChildren<QToolButton*>();
    for (auto* btn : buttons) {
        btn->setChecked(btn == senderBtn);
    }

    QString txt = senderBtn->text();
    if (txt == QStringLiteral("1.5x")) {
        zoomFactor_ = 1.5f;
    } else if (txt == QStringLiteral("2x")) {
        zoomFactor_ = 2.0f;
    } else {
        zoomFactor_ = 1.0f;
    }

    if (!lastImage_.isNull()) {
        setPreview(lastImage_);
    }
}

} // namespace revoscan::app
