#ifndef PCI_BAR0_H
#define PCI_BAR0_H

#include <cstddef>
#include <cstdint>
#include <string>

class PciBar0Mapper {
    friend class P3DecisionBridgeAccess;  // [TEST-ONLY] in-memory BAR injection
public:
    PciBar0Mapper() = default;
    ~PciBar0Mapper();

    PciBar0Mapper(const PciBar0Mapper&) = delete;
    PciBar0Mapper& operator=(const PciBar0Mapper&) = delete;

    bool open(const std::string& bdf, std::size_t bytes = 8192,
              std::string* error = nullptr);
    void close();

    bool is_open() const { return mapping_ != nullptr; }
    volatile std::uint8_t* data() const { return mapping_; }
    std::size_t size() const { return size_; }

private:
    int fd_ = -1;
    volatile std::uint8_t* mapping_ = nullptr;
    std::size_t size_ = 0;
};

#endif  // PCI_BAR0_H
