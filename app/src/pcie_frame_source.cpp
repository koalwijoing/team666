#include "pcie_frame_source.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace {
std::runtime_error system_error(const char* operation)
{
    return std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}
}

PcieFrameSource::PcieFrameSource(const std::string& device, std::uint32_t width,
                                 std::uint32_t height, std::uint32_t pixel_bytes)
{
    fd_ = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0)
        throw system_error("open PCIe device");

    pango_frame_config config{};
    config.abi_version = PANGO_FRAME_ABI_VERSION;
    config.width = width;
    config.height = height;
    config.pixel_bytes = pixel_bytes;
    config.packet_bytes = PANGO_DMA_PACKET_BYTES;
    config.buffer_count = PANGO_FRAME_BUFFER_COUNT;
    if (::ioctl(fd_, PANGO_FRAME_CONFIG_CMD, &config) < 0) {
        const auto error = system_error("configure frame buffers");
        ::close(fd_);
        fd_ = -1;
        throw error;
    }

    if (::ioctl(fd_, PANGO_FRAME_INFO_CMD, &info_) < 0) {
        const auto error = system_error("query frame buffers");
        ::close(fd_);
        fd_ = -1;
        throw error;
    }
    if (info_.abi_version != PANGO_FRAME_ABI_VERSION ||
        info_.buffer_count != PANGO_FRAME_BUFFER_COUNT) {
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error("driver/application frame ABI mismatch");
    }

    void* mapped = ::mmap(nullptr, static_cast<std::size_t>(info_.mmap_bytes),
                          PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mapped == MAP_FAILED) {
        const auto error = system_error("mmap coherent frame buffers");
        ::close(fd_);
        fd_ = -1;
        throw error;
    }
    mapping_ = static_cast<std::uint8_t*>(mapped);
}

PcieFrameSource::~PcieFrameSource()
{
    if (mapping_)
        ::munmap(mapping_, static_cast<std::size_t>(info_.mmap_bytes));
    if (fd_ >= 0)
        ::close(fd_);
}

CaptureResult PcieFrameSource::capture(std::uint32_t buffer_index,
                                       std::uint32_t timeout_ms)
{
    pango_frame_capture request{};
    request.buffer_index = buffer_index;
    request.timeout_ms = timeout_ms;
    if (::ioctl(fd_, PANGO_FRAME_CAPTURE_CMD, &request) < 0)
        throw system_error("capture PCIe frame");
    return {request.frame_sequence, request.bytes_transferred, request.elapsed_ns};
}

const std::uint8_t* PcieFrameSource::buffer(std::uint32_t buffer_index) const
{
    if (buffer_index >= info_.buffer_count)
        throw std::out_of_range("frame buffer index");
    return mapping_ + static_cast<std::size_t>(buffer_index) * info_.slot_bytes;
}

std::uint8_t* PcieFrameSource::buffer(std::uint32_t buffer_index)
{
    return const_cast<std::uint8_t*>(
        static_cast<const PcieFrameSource&>(*this).buffer(buffer_index));
}
