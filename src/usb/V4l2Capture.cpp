#include "V4l2Capture.h"

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <fcntl.h>
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
    const fs::path classRoot("/sys/class/video4linux");
    std::error_code ec;
    if (!fs::is_directory(classRoot, ec)) {
        return std::nullopt;
    }

    for (const auto& entry : fs::directory_iterator(classRoot, ec)) {
        if (ec || !belongsToUsbDevice(entry.path(), vendorId, productId)) {
            continue;
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

V4l2Capture::~V4l2Capture() {
    close();
}

void V4l2Capture::setError(const std::string& operation) {
    std::ostringstream message;
    message << operation << ": " << std::strerror(errno);
    lastError_ = message.str();
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

std::optional<V4l2Frame> V4l2Capture::readFrame(int timeoutMs) {
    if (fd_ < 0 || !streaming_) {
        return std::nullopt;
    }

    pollfd descriptor{fd_, POLLIN, 0};
    int rc;
    do {
        rc = poll(&descriptor, 1, timeoutMs);
    } while (rc < 0 && errno == EINTR);
    if (rc <= 0 || !(descriptor.revents & POLLIN)) {
        if (rc < 0) {
            setError("poll");
        }
        return std::nullopt;
    }

    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
        if (errno != EAGAIN) {
            setError("VIDIOC_DQBUF");
        }
        return std::nullopt;
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
