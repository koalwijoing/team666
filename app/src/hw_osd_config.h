#pragma once
#include <cstring>
inline bool p3_hw_osd_enabled(const char* value) {
    return !(value && (std::strcmp(value,"0")==0 || std::strcmp(value,"false")==0 || std::strcmp(value,"off")==0));
}
