#pragma once

#include <QWidget>

#include "protocol/ScannerProtocol.h"

class QComboBox;
class QLabel;
class QPushButton;
class QSlider;

namespace revoscan::app {

// Ribbon/side controls for POP4 lighting & laser. Disabled until the
// connected protocol reports capture-backed capabilities.
class ScannerControlPanel : public QWidget {
    Q_OBJECT
public:
    explicit ScannerControlPanel(QWidget* parent = nullptr);

    void bindProtocol(protocol::IScannerProtocol* proto);
    void refreshFromProtocol();

signals:
    void controlsChanged();
    void controlError(const QString& message);

private:
    void applyEnabledState();
    QWidget* makeSliderRow(const QString& label, QSlider*& slider, QLabel*& valueLabel);
    void updateSliderValueLabel(QSlider* slider, QLabel* valueLabel);

    protocol::IScannerProtocol* proto_ = nullptr;
    QComboBox* modeBox_ = nullptr;
    QPushButton* alignBtn_ = nullptr;
    QSlider* illumSlider_ = nullptr;
    QLabel* illumValue_ = nullptr;
    QSlider* laserSlider_ = nullptr;
    QLabel* laserValue_ = nullptr;
    QSlider* exposureSlider_ = nullptr;
    QLabel* exposureValue_ = nullptr;
    QPushButton* emittersOffBtn_ = nullptr;
    QLabel* statusLabel_ = nullptr;
};

} // namespace revoscan::app
