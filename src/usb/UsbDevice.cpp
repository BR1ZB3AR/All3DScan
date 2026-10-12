#include "UsbDevice.h"

#include <fstream>

#include <libusb-1.0/libusb.h>
#include <nlohmann/json.hpp>

namespace revoscan::usb {

bool deviceMatches(const DeviceId& id, uint16_t vendorId, uint16_t productId) {
    if (vendorId != id.vendorId) {
        return false;
    }
    return id.matchAnyProduct || productId == id.productId;
}

std::vector<DeviceId> loadKnownDeviceIds(const std::string& jsonPath) {
    std::vector<DeviceId> ids;
    std::ifstream file(jsonPath);
    if (!file.is_open()) {
        return ids;
    }

    try {
        nlohmann::json document;
        file >> document;
        if (!document.contains("devices") || !document["devices"].is_array()) {
            return ids;
        }
        for (const auto& entry : document["devices"]) {
            if (!entry.contains("vendorId")) {
                continue;
            }
            // Entries can be shipped disabled (e.g. unconfirmed IDs); skip them.
            if (!entry.value("enabled", true)) {
                continue;
            }
            DeviceId id;
            id.vendorId = static_cast<uint16_t>(
                std::stoul(entry.at("vendorId").get<std::string>(), nullptr, 16));

            // productId may be absent or "*" to match the whole vendor.
            if (!entry.contains("productId") ||
                entry.at("productId").get<std::string>() == "*") {
                id.matchAnyProduct = true;
                id.productId = 0;
            } else {
                id.productId = static_cast<uint16_t>(
                    std::stoul(entry.at("productId").get<std::string>(), nullptr, 16));
            }
            id.label = entry.value("label", "");
            id.vendorName = entry.value("vendor", "");
            ids.push_back(id);
        }
    } catch (const std::exception&) {
        return {};
    }
    return ids;
}

UsbDevice::UsbDevice() {
    libusb_init(&ctx_);
}

UsbDevice::~UsbDevice() {
    close();
    if (ctx_) {
        libusb_exit(ctx_);
    }
}

bool UsbDevice::openFirstMatching(const std::vector<DeviceId>& knownIds) {
    if (!ctx_) {
        return false;
    }

    libusb_device** list = nullptr;
    ssize_t count = libusb_get_device_list(ctx_, &list);
    if (count < 0) {
        return false;
    }

    bool opened = false;
    for (ssize_t i = 0; i < count && !opened; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(list[i], &desc) != 0) {
            continue;
        }
        for (const auto& id : knownIds) {
            if (deviceMatches(id, desc.idVendor, desc.idProduct)) {
                if (libusb_open(list[i], &handle_) == 0) {
                    opened = true;
                }
                break;
            }
        }
    }

    libusb_free_device_list(list, 1);
    return opened;
}

bool UsbDevice::open(uint16_t vendorId, uint16_t productId) {
    if (!ctx_ || handle_) {
        return false;
    }
    handle_ = libusb_open_device_with_vid_pid(ctx_, vendorId, productId);
    return handle_ != nullptr;
}

void UsbDevice::close() {
    if (handle_) {
        if (claimedInterface_ >= 0) {
            libusb_release_interface(handle_, claimedInterface_);
            claimedInterface_ = -1;
        }
        if (detachedInterface_ >= 0) {
            libusb_attach_kernel_driver(handle_, detachedInterface_);
            detachedInterface_ = -1;
        }
        libusb_close(handle_);
        handle_ = nullptr;
    }
}

int UsbDevice::kernelDriverActive(int interfaceNumber) const {
    if (!handle_) {
        return LIBUSB_ERROR_NO_DEVICE;
    }
    return libusb_kernel_driver_active(handle_, interfaceNumber);
}

int UsbDevice::detachKernelDriver(int interfaceNumber) {
    if (!handle_) {
        return LIBUSB_ERROR_NO_DEVICE;
    }
    const int rc = libusb_detach_kernel_driver(handle_, interfaceNumber);
    if (rc == LIBUSB_SUCCESS) {
        detachedInterface_ = interfaceNumber;
    }
    return rc;
}

int UsbDevice::attachKernelDriver(int interfaceNumber) {
    if (!handle_) {
        return LIBUSB_ERROR_NO_DEVICE;
    }
    const int rc = libusb_attach_kernel_driver(handle_, interfaceNumber);
    if (rc == LIBUSB_SUCCESS && detachedInterface_ == interfaceNumber) {
        detachedInterface_ = -1;
    }
    return rc;
}

bool UsbDevice::claimInterface(int interfaceNumber) {
    if (!handle_) {
        return false;
    }
    if (libusb_claim_interface(handle_, interfaceNumber) != 0) {
        return false;
    }
    claimedInterface_ = interfaceNumber;
    return true;
}

int UsbDevice::releaseInterface() {
    if (!handle_ || claimedInterface_ < 0) {
        return LIBUSB_SUCCESS;
    }
    const int rc = libusb_release_interface(handle_, claimedInterface_);
    if (rc == LIBUSB_SUCCESS) {
        claimedInterface_ = -1;
    }
    return rc;
}

int UsbDevice::resetDevice() {
    if (!handle_) {
        return LIBUSB_ERROR_NO_DEVICE;
    }
    // Drop any claim first so reset is allowed.
    if (claimedInterface_ >= 0) {
        libusb_release_interface(handle_, claimedInterface_);
        claimedInterface_ = -1;
    }
    const int rc = libusb_reset_device(handle_);
    // After reset the handle is invalid on many stacks; always close.
    libusb_close(handle_);
    handle_ = nullptr;
    detachedInterface_ = -1;
    return rc;
}

int UsbDevice::controlTransfer(uint8_t requestType,
                               uint8_t request,
                               uint16_t value,
                               uint16_t index,
                               uint8_t* data,
                               uint16_t length,
                               unsigned int timeoutMs) {
    if (!handle_) {
        return LIBUSB_ERROR_NO_DEVICE;
    }
    return libusb_control_transfer(
        handle_, requestType, request, value, index, data, length, timeoutMs);
}

int UsbDevice::bulkWrite(uint8_t endpoint, const uint8_t* data, int length, int timeoutMs) {
    if (!handle_) {
        return LIBUSB_ERROR_NO_DEVICE;
    }
    int transferred = 0;
    int rc = libusb_bulk_transfer(handle_, endpoint, const_cast<uint8_t*>(data), length, &transferred, timeoutMs);
    return rc == LIBUSB_SUCCESS ? transferred : rc;
}

int UsbDevice::bulkRead(uint8_t endpoint, uint8_t* buffer, int length, int timeoutMs) {
    if (!handle_) {
        return LIBUSB_ERROR_NO_DEVICE;
    }
    int transferred = 0;
    int rc = libusb_bulk_transfer(handle_, endpoint | LIBUSB_ENDPOINT_IN, buffer, length, &transferred, timeoutMs);
    return rc == LIBUSB_SUCCESS ? transferred : rc;
}

} // namespace revoscan::usb
