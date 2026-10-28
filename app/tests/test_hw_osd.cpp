#include "hw_osd_config.h"
#include <cassert>
#include <cstdio>
int main() {
    assert(p3_hw_osd_enabled(nullptr));
    assert(p3_hw_osd_enabled(""));
    assert(p3_hw_osd_enabled("1"));
    assert(p3_hw_osd_enabled("true"));
    assert(p3_hw_osd_enabled("unexpected"));
    assert(!p3_hw_osd_enabled("0"));
    assert(!p3_hw_osd_enabled("false"));
    assert(!p3_hw_osd_enabled("off"));
    std::puts("HW_OSD_TESTS checks=8 failures=0");
}
