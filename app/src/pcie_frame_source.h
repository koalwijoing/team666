#ifndef PCIE_FRAME_SOURCE_H
#define PCIE_FRAME_SOURCE_H

#include "pango_frame_abi.h" // Same production ABI, resolved via explicit build include.

#include <cstddef>
#include <cstdint>
#include <string>

struct CaptureResult {
    std::uint32_t sequence = 0;
    std::uint32_t bytes = 0;
    std::uint64_t device_elapsed_ns = 0;
};

class PcieFrameSource {
public:
    PcieFrameSource(const std::string& device, std::uint32_t width,
                    std::uint32_t height, std::uint32_t pixel_bytes = 2);
    ~PcieFrameSource();

    PcieFrameSource(const PcieFrameSource&) = delete;
    PcieFrameSource& operator=(const PcieFrameSource&) = delete;

    CaptureResult capture(std::uint32_t buffer_index, std::uint32_t timeout_ms);
    const std::uint8_t* buffer(std::uint32_t buffer_index) const;
    std::uint8_t* buffer(std::uint32_t buffer_index);

    std::uint32_t width() const { return info_.width; }
    std::uint32_t height() const { return info_.height; }
    std::size_t frame_bytes() const { return info_.frame_bytes; }
    std::size_t slot_bytes() const { return info_.slot_bytes; }
    std::size_t mmap_bytes() const { return static_cast<std::size_t>(info_.mmap_bytes); }
    std::uint32_t buffer_count() const { return info_.buffer_count; }

private:
    int fd_ = -1;
    std::uint8_t* mapping_ = nullptr;
    pango_frame_info info_{};
};

#endif
