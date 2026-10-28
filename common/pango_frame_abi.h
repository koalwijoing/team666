#ifndef PANGO_FRAME_ABI_H
#define PANGO_FRAME_ABI_H

/* Shared, fixed-width ABI for the RK3568 application and the PCIe driver. */
#include <linux/types.h>
#include <linux/ioctl.h>

#define PANGO_IOC_MAGIC              'S'
#define PANGO_FRAME_ABI_VERSION      1U
#define PANGO_FRAME_BUFFER_COUNT     2U
#define PANGO_DMA_PACKET_BYTES       2048U
#define PANGO_DMA_DONE_BAR           0U
#define PANGO_DMA_DONE_COUNT_OFFSET  0x1f0U
#define PANGO_DMA_STATUS_SIGNATURE_OFFSET 0x1f4U
#define PANGO_DMA_STATUS_SIGNATURE  0x50414e47U /* "PANG" */

struct pango_frame_config {
    __u32 abi_version;
    __u32 width;
    __u32 height;
    __u32 pixel_bytes;
    __u32 packet_bytes;
    __u32 buffer_count;
    __u32 flags;
    __u32 reserved;
};

struct pango_frame_info {
    __u32 abi_version;
    __u32 width;
    __u32 height;
    __u32 pixel_bytes;
    __u32 packet_bytes;
    __u32 buffer_count;
    __u32 frame_bytes;
    __u32 slot_bytes;
    __u64 mmap_bytes;
    __u64 frames_captured;
};

struct pango_frame_capture {
    __u32 buffer_index;
    __u32 timeout_ms;
    __u32 frame_sequence;
    __u32 bytes_transferred;
    __u64 elapsed_ns;
};

#define PANGO_FRAME_CONFIG_CMD  _IOWR(PANGO_IOC_MAGIC, 12, struct pango_frame_config)
#define PANGO_FRAME_INFO_CMD    _IOR (PANGO_IOC_MAGIC, 13, struct pango_frame_info)
#define PANGO_FRAME_CAPTURE_CMD _IOWR(PANGO_IOC_MAGIC, 14, struct pango_frame_capture)

#endif
