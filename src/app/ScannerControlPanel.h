#pragma once

#include <atomic>
#include <thread>

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

    ~ScannerControlPanel() override;

    void bindProtocol(protocol::IScannerProtocol* proto);
    void refreshFromProtocol();

signals:
    void controlsChanged();
    void controlError(const QString& message);

private:
    void applyEnabledState();
    // Mode changes can reboot the scanner (~40 s), so they run off the UI
    // thread with every control disabled meanwhile.
    void startModeChange(protocol::ScanMode mode);
    void finishModeChange(bool ok, const QString& error);
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
    QPushButton* projectorBtn_ = nullptr;
    QPushButton* fillLightBtn_ = nullptr;
    QSlider* fillSlider_ = nullptr;
    QLabel* fillValue_ = nullptr;
    std::thread modeThread_;
    bool switchingMode_ = false;
};

} // namespace revoscan::app
