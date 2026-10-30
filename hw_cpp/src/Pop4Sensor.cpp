#include "all3dscan/Pop4Sensor.h"

#include "decoder/RgbCalibration.h"

#include <stdexcept>
#include <string>

#include "protocol/RevopointPop4Protocol.h"
#include "protocol/ScannerProtocol.h"
#include "usb/UsbDevice.h"

namespace all3dscan::hw {

namespace protocol = revoscan::protocol;
namespace decoder = revoscan::decoder;

namespace {

protocol::Light toLight(uint8_t light) {
    switch (light) {
    case Pop4Sensor::kProjector:
        return protocol::Light::Projector;
    case Pop4Sensor::kFillLight:
        return protocol::Light::FillLight;
    default:
        throw std::invalid_argument("Pop4Sensor: no light " + std::to_string(light));
    }
}

protocol::ScanMode toMode(uint8_t mode) {
    switch (mode) {
    case Pop4Sensor::kInfrared:
        return protocol::ScanMode::StructuredLight;
    case Pop4Sensor::kBlueLaser:
        return protocol::ScanMode::BlueLaser;
    case Pop4Sensor::kSpeckle:
        return protocol::ScanMode::Speckle;
    default:
        throw std::invalid_argument("Pop4Sensor: no scan mode " + std::to_string(mode));
    }
}

} // namespace

Pop4Sensor::Pop4Sensor() : scanner_(protocol::makeRevopointPop4Protocol()) {
    // Before anything claims the depth interface. The file channel and the
    // depth interface cannot both be held, and connecting claims it: doing
    // this afterwards leaves the capture node unusable (VIDIOC_QBUF: No
    // such device), which is how it was found. Failing costs the colour
    // overlay and nothing else, so it never stops a connection.
    readColorCalibration();
    // A live view: show the newest frame, never one that waited in the queue.
    scanner_->setNewestFrameOnly(true);
    if (!scanner_->connect()) {
        throw std::runtime_error("POP 4: " + scanner_->lastError());
    }
    if (!scanner_->startStream()) {
        const std::string reason = scanner_->lastError();
        scanner_->disconnect();
        throw std::runtime_error("POP 4: " + reason);
    }
    // Arming and opening the node can both succeed against a scanner that is
    // on its way out: it re-enumerates after every session, and a descriptor
    // opened just before that delivers nothing and then hangs up. So prove
    // the stream before reporting a connection, rather than leaving the
    // caller to find out frames later.
    if (!captureFirstFrame()) {
        const std::string reason = error_.empty() ? "no frames arrived" : error_;
        scanner_->stopStream();
        scanner_->disconnect();
        throw std::runtime_error("POP 4 connected but sent no frames: " + reason);
    }
    serial_ = scanner_->calibration().serial;
    backend_ = scanner_->backendName();
}

// Waits for the scanner's first frame. The first capture of a session often
// times out while the sensor starts up, so this tries a few times, and stops
// early once the scanner has gone.
bool Pop4Sensor::captureFirstFrame() {
    for (int attempt = 0; attempt < kFirstFrameAttempts; ++attempt) {
        if (capture(kFirstFrameTimeoutMs)) {
            return true;
        }
        if (!scanner_->stillConnected()) {
            return false;
        }
    }
    return false;
}

Pop4Sensor::~Pop4Sensor() {
    // Never let a failure here escape a destructor; the scanner is left as
    // the hardware rules ask (docs: CLAUDE.md) as far as it will go.
    try {
        const bool infrared =
            scanner_->controlState().mode == protocol::ScanMode::StructuredLight;
        if (infrared) {
            if (!scanner_->lightIsOn(protocol::Light::Projector)) {
                scanner_->setLight(protocol::Light::Projector, true);
            }
            if (scanner_->lightIsOn(protocol::Light::FillLight)) {
                scanner_->setLight(protocol::Light::FillLight, false);
            }
        }
        scanner_->stopStream();
        scanner_->disconnect();
        if (!infrared) {
            // Only a reboot returns a POP 4 to infrared; start it and go.
            revoscan::usb::UsbDevice usb;
            if (usb.open(protocol::pop4::kVendorId, protocol::pop4::kProductId)) {
                usb.resetDevice();
            }
        }
    } catch (...) {
    }
}

void Pop4Sensor::readColorCalibration() {
    // The three blobs the vendor stores beside camparam.yaml. They map a
    // point in the rectified left depth frame — which is the frame our
    // points are already in — into the colour camera's image.
    std::string error;
    const auto prgb = protocol::pop4::readDeviceFile("/data/camparam/Prgb.bin", &error);
    const auto distort = protocol::pop4::readDeviceFile("/data/camparam/Distort.bin", &error);
    const auto lcRt = protocol::pop4::readDeviceFile("/data/camparam/LC_RT.bin", &error);
    if (!prgb || !distort || !lcRt) {
        return;
    }
    const auto cal = decoder::parseRgbCalibration(*prgb, *distort, *lcRt, &error);
    if (!cal) {
        return;
    }
    colorLens_ = {cal->fx(), cal->fy(), cal->cx(), cal->cy(),
                  cal->D[0], cal->D[1], cal->D[2], cal->D[3], cal->D[4]};
    colorRotation_.assign(cal->R.begin(), cal->R.end());
    colorTranslation_.assign(cal->T.begin(), cal->T.end());
    colorCalWidth_ = static_cast<uint32_t>(cal->width);
    colorCalHeight_ = static_cast<uint32_t>(cal->height);
    hasColorCal_ = true;
}

bool Pop4Sensor::check(bool ok) {
    if (!ok) {
        error_ = scanner_->lastError();
    }
    return ok;
}

bool Pop4Sensor::capture(int32_t timeoutMs) {
    auto frame = scanner_->readFrame(timeoutMs);
    if (!frame) {
        return check(false);
    }
    const bool metric = frame->encoding == protocol::DepthEncoding::MetricMillimeters;
    if (metric) {
        width_ = frame->width;
        height_ = frame->height;
        const auto lens = scanner_->calibration().depth;
        fx_ = lens.fx;
        fy_ = lens.fy;
        cx_ = lens.cx;
        cy_ = lens.cy;
        depthMm_ = std::move(frame->depthMm);
        depth_ = std::move(frame->depth);
        depthRgb_ = std::move(frame->rgb);
    } else {
        // The packed images came but gave no depth, which is most frames in
        // blue laser mode. Keep the frame, with no depth, for its images: the
        // scanner is still there, and the colour camera shows the laser.
        error_ = scanner_->lastError();
        width_ = kDecodedWidth;
        height_ = kDecodedHeight;
        depthMm_.assign(static_cast<size_t>(width_) * height_, 0);
        depth_.clear();
        depthRgb_.clear();
    }
    const size_t pixels = static_cast<size_t>(width_) * height_;
    depthRgb_.resize(pixels * 3, 0); // no colour camera: black

    points_.clear();
    if (metric) {
        const auto lens = scanner_->calibration().depth;
        if (!(lens.fx > 0.0 && lens.fy > 0.0)) {
            throw std::runtime_error("POP 4: the depth camera has no calibration");
        }
        // The unrounded depth where the decoder gave it; the whole
        // millimetres otherwise, which is what the cloud was always built from.
        const bool fine = depth_.size() == pixels;
        points_.reserve(pixels * 3);
        for (uint32_t v = 0; v < height_; ++v) {
            for (uint32_t u = 0; u < width_; ++u) {
                const size_t at = static_cast<size_t>(v) * width_ + u;
                const float z = fine ? depth_[at] : static_cast<float>(depthMm_[at]);
                if (z <= 0.0f) {
                    continue;
                }
                points_.push_back(static_cast<float>((u - lens.cx) * z / lens.fx));
                points_.push_back(static_cast<float>((v - lens.cy) * z / lens.fy));
                points_.push_back(z);
            }
        }
    }

    const auto infrared = scanner_->lastInfrared();
    if (infrared.width == width_ && infrared.height == height_ && infrared.channels == 1) {
        ir_ = infrared.pixels;
    } else {
        ir_.assign(pixels, 0);
    }
    auto color = scanner_->lastColor();
    if (color.channels == 3 && !color.pixels.empty()) {
        colorWidth_ = color.width;
        colorHeight_ = color.height;
        color_ = std::move(color.pixels);
    }
    ++frameIndex_;
    return true;
}

bool Pop4Sensor::connected() const {
    return scanner_->stillConnected();
}

float Pop4Sensor::validFraction() const {
    return scanner_->depthDecodeStats().validFraction();
}

bool Pop4Sensor::setLight(uint8_t light, bool on) {
    return check(scanner_->setLight(toLight(light), on));
}

bool Pop4Sensor::setLightLevel(uint8_t light, int32_t level) {
    return check(scanner_->setLightLevel(toLight(light), level));
}

bool Pop4Sensor::lightOn(uint8_t light) const {
    return scanner_->lightIsOn(toLight(light));
}

int32_t Pop4Sensor::lightLevel(uint8_t light) const {
    return scanner_->lightLevel(toLight(light));
}

bool Pop4Sensor::setExposure(int32_t microseconds) {
    return check(scanner_->setExposure(microseconds));
}

int32_t Pop4Sensor::exposure() const {
    return scanner_->controlState().exposure;
}

bool Pop4Sensor::setScanMode(uint8_t mode) {
    const bool ok = check(scanner_->setScanMode(toMode(mode)));
    backend_ = scanner_->backendName(); // blue laser has its own calibration
    return ok;
}

uint8_t Pop4Sensor::scanMode() const {
    switch (scanner_->controlState().mode) {
    case protocol::ScanMode::BlueLaser:
        return kBlueLaser;
    case protocol::ScanMode::Speckle:
        return kSpeckle;
    case protocol::ScanMode::StructuredLight:
        break;
    }
    return kInfrared;
}

void Pop4Sensor::setDepthRange(float nearMm, float farMm) {
    scanner_->setDepthRangeHint(nearMm, farMm);
}

std::unique_ptr<Pop4Sensor> openPop4Sensor() {
    return std::make_unique<Pop4Sensor>();
}

} // namespace all3dscan::hw
