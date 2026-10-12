#pragma once

#include <cstdint>
#include <optional>
#include <string>

// Clean-room helpers for the Revopoint WiFi HTTP control channel.
//
// From RevoScan APK 6.0.2.4 (lib3DCamera.so), a scanner in WiFi host mode
// exposes an embedded CGI at:  http://<ip>/cgi-bin/zx_cmd.cgi?<query>
// Observed query strings include get_sn, cam_type=usb&get_exposure,
// cam_type=usb&set_exposure=<us>, cam_type=usb&get_gain, get_farNearState,
// cam_type=usb&get_exposureRange, cam_type=usb&set_aeMaxExp=<v>, and
// set_RedLightIndicator=<0/1>.
//
// These are URL/response helpers only; exact response schemas remain
// best-effort until confirmed by a live WiFi-mode capture.
namespace revoscan::net {

struct ZxHost {
    std::string host = "192.168.3.1";
    uint16_t port = 80;
};

// Full URL for a zx_cmd.cgi query (query must NOT start with '?').
std::string zxCmdUrl(const ZxHost& host, const std::string& query);

// Known command query builders.
std::string zxGetSerial();
std::string zxGetExposure();
std::string zxSetExposure(int microseconds);
std::string zxGetExposureRange();
std::string zxGetGain();
std::string zxSetAeMaxExposure(int value);
std::string zxGetFarNearState();
std::string zxSetRedLightIndicator(bool on);

std::string trim(const std::string& s);

// Best-effort integer extraction from a CGI response body. Handles bare
// integers, key=value, and simple {"...":<int>} JSON-ish payloads.
std::optional<long> parseIntResponse(const std::string& body);

// Extracts a serial-number-like token (first non-empty trimmed line/word).
std::optional<std::string> parseSerialResponse(const std::string& body);

} // namespace revoscan::net
