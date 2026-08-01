#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

struct libusb_context;
struct libusb_device_handle;

namespace revoscan::usb {

struct DeviceId {
    uint16_t vendorId;
    uint16_t productId;
    std::string label;
    // When true, match any product from this vendor (productId is ignored).
    // Set by devices.json entries that omit productId or use "*".
    bool matchAnyProduct = false;
    std::string vendorName;  // "Revopoint", "Creality", "Einstar", ...
};

// Returns true if desc's VID/PID satisfies id (respecting matchAnyProduct).
bool deviceMatches(const DeviceId& id, uint16_t vendorId, uint16_t productId);

// Known/candidate VID:PID pairs for supported scanners. Populated from
// devices.json at runtime (see resources/devices.json) so new IDs can be
// added without recompiling. Covers Revopoint (2207), Creality (2bc5), and
// Einstar/Shining3D families; entries may match a whole vendor. Prefer
// util::findDevicesJson() so .deb / AppImage / source-tree layouts all work.
std::vector<DeviceId> loadKnownDeviceIds(const std::string& jsonPath);

// Thin RAII wrapper around libusb for enumerating and talking to a scanner.
// Kept deliberately small: claim one interface, do bulk transfers. Command
// framing/semantics live in src/protocol, not here.
class UsbDevice {
public:
    UsbDevice();
    ~UsbDevice();

    UsbDevice(const UsbDevice&) = delete;
    UsbDevice& operator=(const UsbDevice&) = delete;

    // Scans connected USB devices for the first match against knownIds.
    bool openFirstMatching(const std::vector<DeviceId>& knownIds);
    bool open(uint16_t vendorId, uint16_t productId);
    void close();
    bool isOpen() const { return handle_ != nullptr; }

    // Kernel-driver operations are needed temporarily for vendor UVC XU
    // controls. Callers must reattach the driver before opening V4L2.
    int kernelDriverActive(int interfaceNumber) const;
    int detachKernelDriver(int interfaceNumber);
    int attachKernelDriver(int interfaceNumber);
    bool claimInterface(int interfaceNumber);
    int releaseInterface();

    // Returns bytes transferred, or a negative libusb error code.
    int controlTransfer(uint8_t requestType,
                        uint8_t request,
                        uint16_t value,
                        uint16_t index,
                        uint8_t* data,
                        uint16_t length,
                        unsigned int timeoutMs);

    // Returns bytes transferred, or a negative libusb error code.
    int bulkWrite(uint8_t endpoint, const uint8_t* data, int length, int timeoutMs);
    int bulkRead(uint8_t endpoint, uint8_t* buffer, int length, int timeoutMs);

private:
    libusb_context* ctx_ = nullptr;
    libusb_device_handle* handle_ = nullptr;
    int claimedInterface_ = -1;
    int detachedInterface_ = -1;
};

} // namespace revoscan::usb
