#include "pci_bar0.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace {
std::string system_error(const char* operation)
{
    return std::string(operation) + ": " + std::strerror(errno);
}
}

PciBar0Mapper::~PciBar0Mapper()
{
    close();
}

bool PciBar0Mapper::open(const std::string& bdf, std::size_t bytes,
                         std::string* error)
{
    close();
    const std::string path =
        "/sys/bus/pci/devices/" + bdf + "/resource0";
    fd_ = ::open(path.c_str(), O_RDWR | O_SYNC | O_CLOEXEC);
    if (fd_ < 0) {
        if (error) *error = system_error("open PCI BAR0 resource");
        return false;
    }

    void* mapped = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd_, 0);
    if (mapped == MAP_FAILED) {
        if (error) *error = system_error("mmap PCI BAR0 resource");
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    mapping_ = static_cast<volatile std::uint8_t*>(mapped);
    size_ = bytes;
    return true;
}

void PciBar0Mapper::close()
{
    if (mapping_) {
        ::munmap(const_cast<std::uint8_t*>(mapping_), size_);
        mapping_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    size_ = 0;
}
