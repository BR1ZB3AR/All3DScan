#include "ScannerControlPanel.h"

#include <QApplication>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QMetaObject>
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
    modeBox_->addItem(tr("Infrared (default)"),
                      static_cast<int>(protocol::ScanMode::StructuredLight));
    modeBox_->addItem(tr("Blue laser (visible)"),
                      static_cast<int>(protocol::ScanMode::BlueLaser));
    modeBox_->addItem(tr("Infrared speckle (sharper)"),
                      static_cast<int>(protocol::ScanMode::Speckle));
    modeBox_->setToolTip(
        tr("Blue laser: the scanner's visible blue laser lines.\n"
           "Infrared speckle: invisible, but a much stronger pattern — about\n"
           "20-30% more depth points.\n"
           "Going back to infrared reboots the scanner (about 45 seconds)."));
    modeV->addWidget(modeBox_);
    root->addWidget(modeSection);

    // Section: built-in lights (verified device commands)
    auto* lightsLabel = new QLabel(tr("Lights"), this);
    lightsLabel->setObjectName("FieldLabel");
    root->addWidget(lightsLabel);

    projectorBtn_ = new QPushButton(tr("Pattern projector"), this);
    projectorBtn_->setCheckable(true);
    projectorBtn_->setToolTip(tr("The infrared projector that draws the scan pattern."));
    root->addWidget(projectorBtn_);

    fillLightBtn_ = new QPushButton(tr("Fill light"), this);
    fillLightBtn_->setCheckable(true);
    fillLightBtn_->setToolTip(
        tr("White LEDs for the colour camera. RevoScan turns these on for\n"
           "marker scanning."));
    root->addWidget(fillLightBtn_);

    root->addWidget(makeSliderRow(tr("Fill light brightness"), fillSlider_, fillValue_));

    // Keep the alignment guide / CS7 rows hidden: unlike the above they are
    // not confirmed against the scanner.
    alignBtn_ = new QPushButton(tr("Alignment Light"), this);
    alignBtn_->setCheckable(true);
    alignBtn_->hide();
    illumSlider_ = new QSlider(Qt::Horizontal, this);
    illumSlider_->hide();
    laserSlider_ = new QSlider(Qt::Horizontal, this);
    laserSlider_->hide();

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
        if (!proto_ || switchingMode_) return;
        startModeChange(static_cast<protocol::ScanMode>(modeBox_->currentData().toInt()));
    });
    connect(projectorBtn_, &QPushButton::toggled, this, [this](bool on) {
        if (!proto_) return;
        if (!proto_->setLight(protocol::Light::Projector, on)) {
            emit controlError(QString::fromStdString(proto_->lastError()));
        }
        refreshFromProtocol();
    });
    connect(fillLightBtn_, &QPushButton::toggled, this, [this](bool on) {
        if (!proto_) return;
        if (!proto_->setLight(protocol::Light::FillLight, on)) {
            emit controlError(QString::fromStdString(proto_->lastError()));
        }
        refreshFromProtocol();
    });
    connect(fillSlider_, &QSlider::sliderReleased, this, [this]() {
        if (!proto_) return;
        if (!proto_->setLightLevel(protocol::Light::FillLight, fillSlider_->value())) {
            emit controlError(QString::fromStdString(proto_->lastError()));
        }
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

void ScannerControlPanel::startModeChange(protocol::ScanMode mode) {
    switchingMode_ = true;
    applyEnabledState();
    statusLabel_->setText(
        mode == protocol::ScanMode::StructuredLight
            ? tr("Back to infrared: the scanner reboots, about 45 seconds…")
            : tr("Switching the scanner's projector…"));
    emit scanModeRequested(static_cast<int>(mode));
}

void ScannerControlPanel::finishModeChange(bool ok, const QString& error) {
    switchingMode_ = false;
    if (!ok) {
        emit controlError(error.isEmpty() ? tr("Scan mode change failed") : error);
    }
    refreshFromProtocol();
    emit controlsChanged();
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
    setSlider(exposureSlider_, exposureValue_, state.exposure);

    if (proto_->supportsLight(protocol::Light::Projector)) {
        fillSlider_->setRange(0, 255);
        projectorBtn_->blockSignals(true);
        projectorBtn_->setChecked(proto_->lightIsOn(protocol::Light::Projector));
        projectorBtn_->blockSignals(false);
        fillLightBtn_->blockSignals(true);
        fillLightBtn_->setChecked(proto_->lightIsOn(protocol::Light::FillLight));
        fillLightBtn_->blockSignals(false);
        setSlider(fillSlider_, fillValue_, proto_->lightLevel(protocol::Light::FillLight));
    }

    if (switchingMode_) {
        return; // keep the "switching…" message
    }
    if (proto_->state() == protocol::ConnectionState::Disconnected) {
        statusLabel_->setText(tr("Connect the scanner to use these controls."));
        return;
    }
    if (proto_->supportsLight(protocol::Light::Projector)) {
        switch (state.mode) {
        case protocol::ScanMode::BlueLaser:
            statusLabel_->setText(
                tr("Blue laser on, using the scanner's blue calibration."));
            break;
        case protocol::ScanMode::Speckle:
            statusLabel_->setText(tr("Infrared speckle: stronger pattern, nothing visible."));
            break;
        case protocol::ScanMode::StructuredLight:
            statusLabel_->setText(
                tr("Infrared mode. The projector and fill light are the scanner's own."));
            break;
        }
    } else {
        statusLabel_->setText(
            tr("Lighting controls need a connected POP4."));
    }
}

void ScannerControlPanel::applyEnabledState() {
    const auto caps = proto_ ? proto_->controlCapabilities()
                             : protocol::ScannerControlCapabilities{};
    // Every one of these talks to the scanner, so none of them may be offered
    // before it is actually connected: the window used to let you pick a scan
    // mode while Connect was still running, and the choice was silently
    // reverted.
    const bool live =
        proto_ && proto_->state() != protocol::ConnectionState::Disconnected && !switchingMode_;
    const bool lights = live && proto_->supportsLight(protocol::Light::Projector);
    modeBox_->setEnabled(live && caps.scanMode);
    exposureSlider_->setEnabled(live && caps.exposure);
    emittersOffBtn_->setEnabled(live && caps.emitterOff);
    projectorBtn_->setEnabled(lights);
    fillLightBtn_->setEnabled(lights);
    fillSlider_->setEnabled(lights);
}

} // namespace revoscan::app
