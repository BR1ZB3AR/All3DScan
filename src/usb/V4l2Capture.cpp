#include "V4l2Capture.h"

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

#include <fcntl.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace revoscan::usb {
namespace {

namespace fs = std::filesystem;

int xioctl(int fd, unsigned long request, void* arg) {
    int rc;
    do {
        rc = ioctl(fd, request, arg);
    } while (rc < 0 && errno == EINTR);
    return rc;
}

std::optional<unsigned long> readHex(const fs::path& path) {
    std::ifstream in(path);
    std::string value;
    if (!(in >> value)) {
        return std::nullopt;
    }
    try {
        return std::stoul(value, nullptr, 16);
    } catch (...) {
        return std::nullopt;
    }
}

bool belongsToUsbDevice(const fs::path& videoClassPath,
                        uint16_t vendorId,
                        uint16_t productId) {
    std::error_code ec;
    fs::path current = fs::canonical(videoClassPath / "device", ec);
    if (ec) {
        return false;
    }

    while (!current.empty() && current != current.root_path()) {
        const auto vid = readHex(current / "idVendor");
        const auto pid = readHex(current / "idProduct");
        if (vid && pid) {
            return *vid == vendorId && *pid == productId;
        }
        current = current.parent_path();
    }
    return false;
}

bool supportsFormat(int fd,
                    uint32_t pixelFormat,
                    uint32_t width,
                    uint32_t height) {
    v4l2_capability caps{};
    if (xioctl(fd, VIDIOC_QUERYCAP, &caps) < 0) {
        return false;
    }
    const uint32_t deviceCaps =
        (caps.capabilities & V4L2_CAP_DEVICE_CAPS) ? caps.device_caps : caps.capabilities;
    if (!(deviceCaps & V4L2_CAP_VIDEO_CAPTURE) ||
        !(deviceCaps & V4L2_CAP_STREAMING)) {
        return false;
    }

    v4l2_fmtdesc format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (format.index = 0; xioctl(fd, VIDIOC_ENUM_FMT, &format) == 0; ++format.index) {
        if (format.pixelformat != pixelFormat) {
            continue;
        }

        v4l2_frmsizeenum size{};
        size.pixel_format = pixelFormat;
        for (size.index = 0; xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size) == 0; ++size.index) {
            if (size.type == V4L2_FRMSIZE_TYPE_DISCRETE &&
                size.discrete.width == width && size.discrete.height == height) {
                return true;
            }
            if ((size.type == V4L2_FRMSIZE_TYPE_STEPWISE ||
                 size.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) &&
                width >= size.stepwise.min_width && width <= size.stepwise.max_width &&
                height >= size.stepwise.min_height && height <= size.stepwise.max_height) {
                return true;
            }
        }
    }
    return false;
}

} // namespace

std::optional<std::string> findV4l2CaptureNode(uint16_t vendorId,
                                               uint16_t productId,
                                               uint32_t pixelFormat,
                                               uint32_t width,
                                               uint32_t height) {
    return findV4l2CaptureNodeNamed(vendorId, productId, pixelFormat, width,
                                    height, {});
}

std::optional<std::string> findV4l2CaptureNodeNamed(uint16_t vendorId,
                                                    uint16_t productId,
                                                    uint32_t pixelFormat,
                                                    uint32_t width,
                                                    uint32_t height,
                                                    const std::string& nameSubstring) {
    const fs::path classRoot("/sys/class/video4linux");
    std::error_code ec;
    if (!fs::is_directory(classRoot, ec)) {
        return std::nullopt;
    }

    for (const auto& entry : fs::directory_iterator(classRoot, ec)) {
        if (ec || !belongsToUsbDevice(entry.path(), vendorId, productId)) {
            continue;
        }
        if (!nameSubstring.empty()) {
            std::ifstream nameFile(entry.path() / "name");
            std::string name;
            std::getline(nameFile, name);
            if (name.find(nameSubstring) == std::string::npos) {
                continue;
            }
        }
        const std::string node = "/dev/" + entry.path().filename().string();
        const int fd = ::open(node.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0) {
            continue;
        }
        const bool supported = supportsFormat(fd, pixelFormat, width, height);
        ::close(fd);
        if (supported) {
            return node;
        }
    }
    return std::nullopt;
}

bool rebindUvcvideoForUsbDevice(uint16_t vendorId, uint16_t productId) {
    const fs::path usbRoot("/sys/bus/usb/devices");
    const fs::path unbindPath("/sys/bus/usb/drivers/uvcvideo/unbind");
    const fs::path bindPath("/sys/bus/usb/drivers/uvcvideo/bind");
    std::error_code ec;
    if (!fs::is_directory(usbRoot, ec) || !fs::exists(unbindPath, ec) ||
        !fs::exists(bindPath, ec)) {
        return false;
    }

    std::vector<std::string> ifaceNames;
    for (const auto& entry : fs::directory_iterator(usbRoot, ec)) {
        if (ec || !fs::is_directory(entry.path(), ec)) {
            continue;
        }
        // Only look at device roots that carry idVendor/idProduct.
        const auto vid = readHex(entry.path() / "idVendor");
        const auto pid = readHex(entry.path() / "idProduct");
        if (!vid || !pid || *vid != vendorId || *pid != productId) {
            continue;
        }
        // Collect Video Control interfaces (class 0e / subclass 01) bound to
        // uvcvideo — binding the VC interface re-attaches the whole IAD.
        for (const auto& child : fs::directory_iterator(entry.path(), ec)) {
            if (ec || !fs::is_directory(child.path(), ec)) {
                continue;
            }
            const auto name = child.path().filename().string();
            if (name.find(':') == std::string::npos) {
                continue;
            }
            const auto ifaceClass = readHex(child.path() / "bInterfaceClass");
            const auto ifaceSub = readHex(child.path() / "bInterfaceSubClass");
            if (!ifaceClass || *ifaceClass != 0x0e || !ifaceSub ||
                *ifaceSub != 0x01) {
                continue;
            }
            const fs::path driver = child.path() / "driver";
            if (!fs::exists(driver, ec)) {
                // Already unbound — still try bind later.
                ifaceNames.push_back(name);
                continue;
            }
            std::error_code linkEc;
            const auto target = fs::read_symlink(driver, linkEc);
            if (linkEc || target.filename() != "uvcvideo") {
                continue;
            }
            ifaceNames.push_back(name);
        }
    }

    if (ifaceNames.empty()) {
        return false;
    }

    auto writeSysfs = [](const fs::path& path, const std::string& value) {
        std::ofstream out(path);
        if (!out) {
            return false;
        }
        out << value;
        return static_cast<bool>(out);
    };

    bool wrote = false;
    for (const auto& name : ifaceNames) {
        // Unbind is best-effort (may already be unbound).
        writeSysfs(unbindPath, name);
    }
    // Brief pause so the kernel finishes tearing down the old video nodes.
    ::usleep(200000);
    for (const auto& name : ifaceNames) {
        if (writeSysfs(bindPath, name)) {
            wrote = true;
        }
    }
    ::usleep(400000);
    return wrote;
}

V4l2Capture::~V4l2Capture() {
    close();
}

void V4l2Capture::setError(const std::string& operation) {
    std::ostringstream message;
    message << operation << ": " << std::strerror(errno);
    lastError_ = message.str();
}

bool uvcExtensionSetCur(int fd, uint8_t unit, uint8_t selector, const uint8_t* data,
                        uint16_t size, std::string* error) {
    std::vector<uint8_t> buf(data, data + size);
    uvc_xu_control_query q{};
    q.unit = unit;
    q.selector = selector;
    q.query = UVC_SET_CUR;
    q.size = size;
    q.data = buf.data();
    if (fd < 0 || ::ioctl(fd, UVCIOC_CTRL_QUERY, &q) != 0) {
        if (error) {
            *error = std::string("UVCIOC_CTRL_QUERY SET_CUR failed: ") +
                     (fd < 0 ? "no open video node" : std::strerror(errno));
        }
        return false;
    }
    return true;
}

bool V4l2Capture::open(const std::string& path,
                       uint32_t pixelFormat,
                       uint32_t width,
                       uint32_t height,
                       uint32_t bufferCount) {
    close();
    fd_ = ::open(path.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
        setError("open " + path);
        return false;
    }

    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = pixelFormat;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
        setError("VIDIOC_S_FMT");
        close();
        return false;
    }
    if (format.fmt.pix.width != width || format.fmt.pix.height != height ||
        format.fmt.pix.pixelformat != pixelFormat) {
        lastError_ = "V4L2 driver substituted an unsupported capture format";
        close();
        return false;
    }

    // POP4 advertises packed Y16 modes at up to 15 fps. Some uvcvideo
    // versions leave the interval from the preceding mode selected unless it
    // is set explicitly, which can result in STREAMON succeeding but no URBs
    // completing. Treat unsupported S_PARM as non-fatal.
    v4l2_streamparm streamParm{};
    streamParm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamParm.parm.capture.timeperframe.numerator = 1;
    streamParm.parm.capture.timeperframe.denominator = 15;
    xioctl(fd_, VIDIOC_S_PARM, &streamParm);

    v4l2_requestbuffers request{};
    request.count = bufferCount;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &request) < 0 || request.count < 2) {
        setError("VIDIOC_REQBUFS");
        close();
        return false;
    }

    buffers_.resize(request.count);
    for (uint32_t i = 0; i < request.count; ++i) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
            setError("VIDIOC_QUERYBUF");
            close();
            return false;
        }
        void* address =
            mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buffer.m.offset);
        if (address == MAP_FAILED) {
            buffers_[i].address = nullptr;
            setError("mmap");
            close();
            return false;
        }
        buffers_[i] = {address, buffer.length};
    }
    return true;
}

bool V4l2Capture::start() {
    if (fd_ < 0 || streaming_) {
        return fd_ >= 0;
    }
    for (uint32_t i = 0; i < buffers_.size(); ++i) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
            setError("VIDIOC_QBUF");
            return false;
        }
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        setError("VIDIOC_STREAMON");
        return false;
    }
    streaming_ = true;
    return true;
}

void V4l2Capture::stop() {
    if (fd_ >= 0 && streaming_) {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_STREAMOFF, &type);
        streaming_ = false;
    }
}

void V4l2Capture::close() {
    stop();
    deviceGone_ = false;
    for (auto& buffer : buffers_) {
        if (buffer.address) {
            munmap(buffer.address, buffer.length);
        }
    }
    buffers_.clear();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

std::optional<V4l2Frame> V4l2Capture::readFrame(int timeoutMs, bool newest) {
    if (fd_ < 0 || !streaming_) {
        return std::nullopt;
    }

    pollfd descriptor{fd_, POLLIN, 0};
    int rc;
    do {
        rc = poll(&descriptor, 1, timeoutMs);
    } while (rc < 0 && errno == EINTR);
    if (rc == 0) {
        lastError_ = "V4L2 frame timeout";
        return std::nullopt;
    }
    if (rc < 0) {
        setError("poll");
        return std::nullopt;
    }
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        // The scanner unplugged or re-enumerated: this descriptor is dead,
        // and a caller that keeps polling it would spin forever.
        deviceGone_ = true;
        std::ostringstream message;
        message << "the scanner disconnected (poll revents=0x" << std::hex
                << descriptor.revents << ')';
        lastError_ = message.str();
        return std::nullopt;
    }
    if (!(descriptor.revents & POLLIN)) {
        return std::nullopt;
    }

    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
        if (errno != EAGAIN) {
            deviceGone_ = deviceGone_ || errno == ENODEV;
            setError("VIDIOC_DQBUF");
        }
        return std::nullopt;
    }
    // The descriptor is non-blocking, so a dequeue with nothing ready fails
    // with EAGAIN, which ends the walk to the newest frame.
    while (newest) {
        v4l2_buffer next{};
        next.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        next.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_, VIDIOC_DQBUF, &next) < 0) {
            break;
        }
        if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
            setError("VIDIOC_QBUF");
            return std::nullopt;
        }
        buffer = next;
    }

    std::optional<V4l2Frame> result;
    if (buffer.index < buffers_.size() &&
        buffer.bytesused <= buffers_[buffer.index].length) {
        const auto* first = static_cast<const uint8_t*>(buffers_[buffer.index].address);
        V4l2Frame frame;
        frame.bytes.assign(first, first + buffer.bytesused);
        frame.timestampUs = static_cast<uint64_t>(buffer.timestamp.tv_sec) * 1000000ULL +
                            static_cast<uint64_t>(buffer.timestamp.tv_usec);
        result = std::move(frame);
    }

    if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
        setError("VIDIOC_QBUF");
        return std::nullopt;
    }
    return result;
}

} // namespace revoscan::usb
