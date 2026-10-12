#include "ZxCmdProtocol.h"

#include <cctype>
#include <cstdlib>
#include <string>

namespace revoscan::net {
namespace {

std::string commandBase() {
    return "/cgi-bin/zx_cmd.cgi?";
}

} // namespace

std::string zxCmdUrl(const ZxHost& host, const std::string& query) {
    std::string url = "http://";
    url += host.host;
    if (host.port != 80) {
        url += ":";
        url += std::to_string(host.port);
    }
    url += commandBase();
    url += query;
    return url;
}

std::string zxGetSerial() {
    return "get_sn";
}

std::string zxGetExposure() {
    return "cam_type=usb&get_exposure";
}

std::string zxSetExposure(int microseconds) {
    if (microseconds < 0) {
        microseconds = 0;
    }
    return "cam_type=usb&set_exposure=" + std::to_string(microseconds);
}

std::string zxGetExposureRange() {
    return "cam_type=usb&get_exposureRange";
}

std::string zxGetGain() {
    return "cam_type=usb&get_gain";
}

std::string zxSetAeMaxExposure(int value) {
    if (value < 0) {
        value = 0;
    }
    return "cam_type=usb&set_aeMaxExp=" + std::to_string(value);
}

std::string zxGetFarNearState() {
    return "get_farNearState";
}

std::string zxSetRedLightIndicator(bool on) {
    return std::string("set_RedLightIndicator=") + (on ? "1" : "0");
}

std::string trim(const std::string& s) {
    size_t start = 0;
    size_t end = s.size();
    while (start < end && std::isspace(static_cast<unsigned char>(s[start]))) {
        ++start;
    }
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
        --end;
    }
    return s.substr(start, end - start);
}

std::optional<long> parseIntResponse(const std::string& body) {
    // Scan for the first signed integer run anywhere in the body. This copes
    // with bare "123", "value=123", and {"exposure":123} without committing to
    // a schema we have not yet confirmed from a live capture.
    const std::string t = trim(body);
    for (size_t i = 0; i < t.size(); ++i) {
        const char c = t[i];
        const bool digit = std::isdigit(static_cast<unsigned char>(c)) != 0;
        const bool signedStart = (c == '-' || c == '+') && i + 1 < t.size() &&
                                 std::isdigit(static_cast<unsigned char>(t[i + 1]));
        if (digit || signedStart) {
            char* endPtr = nullptr;
            const long value = std::strtol(t.c_str() + i, &endPtr, 10);
            if (endPtr != t.c_str() + i) {
                return value;
            }
        }
    }
    return std::nullopt;
}

std::optional<std::string> parseSerialResponse(const std::string& body) {
    const std::string t = trim(body);
    if (t.empty()) {
        return std::nullopt;
    }
    // If JSON-ish (e.g. {"sn":"ABC123"}), pull the quoted token; otherwise
    // return the first whitespace-delimited token.
    const size_t quote = t.find('"');
    if (quote != std::string::npos) {
        const size_t close = t.find('"', quote + 1);
        // Skip a leading key like "sn" and its closing quote; find the value.
        size_t valOpen = t.find('"', close + 1);
        if (valOpen != std::string::npos) {
            const size_t valClose = t.find('"', valOpen + 1);
            if (valClose != std::string::npos && valClose > valOpen + 1) {
                return t.substr(valOpen + 1, valClose - valOpen - 1);
            }
        }
        if (close != std::string::npos && close > quote + 1) {
            return t.substr(quote + 1, close - quote - 1);
        }
    }
    std::string token;
    for (char c : t) {
        if (std::isspace(static_cast<unsigned char>(c))) {
            break;
        }
        token.push_back(c);
    }
    if (token.empty()) {
        return std::nullopt;
    }
    return token;
}

} // namespace revoscan::net
