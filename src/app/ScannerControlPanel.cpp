#include "ScannerControlPanel.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>

namespace revoscan::app {

QWidget* ScannerControlPanel::makeSliderRow(const QString& label, QSlider*& slider, QLabel*& valueLabel) {
    auto* row = new QWidget(this);
    auto* v = new QVBoxLayout(row);
    v->setContentsMargins(0, 2, 0, 2);
    v->setSpacing(2);

    auto* topRow = new QHBoxLayout();
    topRow->setContentsMargins(0, 0, 0, 0);
    auto* nameLabel = new QLabel(label, row);
    nameLabel->setObjectName("FieldLabel");
    topRow->addWidget(nameLabel);
    topRow->addStretch(1);

    valueLabel = new QLabel(QStringLiteral("0"), row);
    valueLabel->setFixedWidth(30);
    valueLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    valueLabel->setStyleSheet("color: #8b9198; font-size: 11px;");
    topRow->addWidget(valueLabel);
    v->addLayout(topRow);

    slider = new QSlider(Qt::Horizontal, row);
    v->addWidget(slider);

    connect(slider, &QSlider::valueChanged, this, [this, slider, valueLabel]() {
        updateSliderValueLabel(slider, valueLabel);
    });

    return row;
}

void ScannerControlPanel::updateSliderValueLabel(QSlider* slider, QLabel* valueLabel) {
    if (!slider || !valueLabel) return;
    // Show raw slider value; in a real protocol the value maps to the full
    // range defined by controlCapabilities().
    const int pct = slider->maximum() > 0
        ? (slider->value() * 100) / slider->maximum()
        : 0;
    valueLabel->setText(QStringLiteral("%1%").arg(pct));
}

ScannerControlPanel::ScannerControlPanel(QWidget* parent) : QWidget(parent) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(6);

    auto* title = new QLabel(tr("Scanner Controls"), this);
    title->setObjectName("PanelTitle");
    root->addWidget(title);

    // Section: Scan Mode
    auto* modeSection = new QWidget(this);
    auto* modeV = new QVBoxLayout(modeSection);
    modeV->setContentsMargins(0, 0, 0, 0);
    modeV->setSpacing(4);
    modeV->addWidget(new QLabel(tr("Scan Mode"), this));
    modeBox_ = new QComboBox(this);
    modeBox_->addItem(tr("Structured Light"),
                      static_cast<int>(protocol::ScanMode::StructuredLight));
    modeBox_->addItem(tr("Blue Laser"), static_cast<int>(protocol::ScanMode::BlueLaser));
    modeV->addWidget(modeBox_);
    root->addWidget(modeSection);

    // Section: Alignment Light
    alignBtn_ = new QPushButton(tr("Alignment Light"), this);
    alignBtn_->setCheckable(true);
    root->addWidget(alignBtn_);

    // Section: Illumination
    root->addWidget(makeSliderRow(tr("Illumination"), illumSlider_, illumValue_));

    // Section: Scan Laser
    root->addWidget(makeSliderRow(tr("Scan Laser"), laserSlider_, laserValue_));

    // Section: Exposure
    root->addWidget(makeSliderRow(tr("Exposure"), exposureSlider_, exposureValue_));

    // Emitters Off button
    emittersOffBtn_ = new QPushButton(tr("Emitters Off"), this);
    root->addWidget(emittersOffBtn_);

    // Status label
    statusLabel_ = new QLabel(tr("Controls unavailable until capture-backed"), this);
    statusLabel_->setWordWrap(true);
    statusLabel_->setObjectName("StatusHint");
    root->addWidget(statusLabel_);
    root->addStretch(1);

    // Connections
    connect(modeBox_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
        if (!proto_) return;
        const auto mode = static_cast<protocol::ScanMode>(modeBox_->currentData().toInt());
        if (!proto_->setScanMode(mode)) {
            emit controlError(QString::fromStdString(proto_->lastError()));
        }
        emit controlsChanged();
    });
    connect(alignBtn_, &QPushButton::toggled, this, [this](bool on) {
        if (!proto_) return;
        if (!proto_->setAlignmentGuide(on)) {
            emit controlError(QString::fromStdString(proto_->lastError()));
            refreshFromProtocol();
        }
        emit controlsChanged();
    });
    connect(illumSlider_, &QSlider::sliderReleased, this, [this]() {
        if (!proto_) return;
        if (!proto_->setIlluminationPower(illumSlider_->value())) {
            emit controlError(QString::fromStdString(proto_->lastError()));
        }
    });
    connect(laserSlider_, &QSlider::sliderReleased, this, [this]() {
        if (!proto_) return;
        if (!proto_->setLaserPower(laserSlider_->value())) {
            emit controlError(QString::fromStdString(proto_->lastError()));
        }
    });
    connect(exposureSlider_, &QSlider::sliderReleased, this, [this]() {
        if (!proto_) return;
        if (!proto_->setExposure(exposureSlider_->value())) {
            emit controlError(QString::fromStdString(proto_->lastError()));
        }
    });
    connect(emittersOffBtn_, &QPushButton::clicked, this, [this] {
        if (!proto_) return;
        if (!proto_->emittersOff()) {
            emit controlError(QString::fromStdString(proto_->lastError()));
        }
        refreshFromProtocol();
        emit controlsChanged();
    });

    applyEnabledState();
}

void ScannerControlPanel::bindProtocol(protocol::IScannerProtocol* proto) {
    proto_ = proto;
    refreshFromProtocol();
}

void ScannerControlPanel::refreshFromProtocol() {
    applyEnabledState();
    if (!proto_) return;

    const auto caps = proto_->controlCapabilities();
    const auto state = proto_->controlState();
    illumSlider_->setRange(caps.illuminationMin, std::max(caps.illuminationMax, 1));
    laserSlider_->setRange(caps.laserMin, std::max(caps.laserMax, 1));
    exposureSlider_->setRange(caps.exposureMin, std::max(caps.exposureMax, 1));

    const int modeIndex = modeBox_->findData(static_cast<int>(state.mode));
    if (modeIndex >= 0) {
        modeBox_->blockSignals(true);
        modeBox_->setCurrentIndex(modeIndex);
        modeBox_->blockSignals(false);
    }
    alignBtn_->blockSignals(true);
    alignBtn_->setChecked(state.alignmentGuideOn);
    alignBtn_->blockSignals(false);

    auto setSlider = [](QSlider* slider, QLabel* valueLbl, int value) {
        slider->blockSignals(true);
        slider->setValue(value);
        slider->blockSignals(false);
        if (valueLbl) {
            const int pct = slider->maximum() > 0
                ? (value * 100) / slider->maximum()
                : 0;
            valueLbl->setText(QStringLiteral("%1%").arg(pct));
        }
    };
    setSlider(illumSlider_, illumValue_, state.illuminationPower);
    setSlider(laserSlider_, laserValue_, state.laserPower);
    setSlider(exposureSlider_, exposureValue_, state.exposure);

    if (caps.scanMode || caps.alignmentGuide || caps.illuminationPower ||
        caps.laserPower || caps.exposure) {
        statusLabel_->setText(
            tr("POP4 Projector, Flash Light & Blue Laser Controls Active (CS7/CS10 UVC XU)."));
    } else {
        statusLabel_->setText(
            tr("POP4 lighting / blue-laser controls need USBPcap-confirmed "
               "commands (see docs/protocol/USBCAP_CHECKLIST.md)"));
    }
}

void ScannerControlPanel::applyEnabledState() {
    const auto caps = proto_ ? proto_->controlCapabilities()
                             : protocol::ScannerControlCapabilities{};
    modeBox_->setEnabled(caps.scanMode);
    alignBtn_->setEnabled(caps.alignmentGuide);
    illumSlider_->setEnabled(caps.illuminationPower);
    laserSlider_->setEnabled(caps.laserPower);
    exposureSlider_->setEnabled(caps.exposure);
    emittersOffBtn_->setEnabled(caps.emitterOff);
}

} // namespace revoscan::app
