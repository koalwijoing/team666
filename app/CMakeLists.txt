cmake_minimum_required(VERSION 3.16)
project(HuiMuADAS LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_C_FLAGS_RELEASE "-O2 -ffp-contract=off" CACHE STRING "Release C flags" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -ffp-contract=off" CACHE STRING "Release C++ flags" FORCE)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(OPENCV REQUIRED IMPORTED_TARGET opencv4)
pkg_check_modules(GTK REQUIRED IMPORTED_TARGET gtk+-2.0)

set(DEPLOY_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../deployment/lib")
set(APP_INCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/includes"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/common"
    "${CMAKE_CURRENT_SOURCE_DIR}/sdk")
set(APP_SOURCES
    src/main.cpp src/crosswalk_person.cpp src/twinlite_lane_model.cpp
    src/p3_engine.cpp src/candidate_postprocess.cpp src/lane_shared_frontend.cpp
    src/twinlite_dash.cpp src/p3_decision_bridge.cpp src/decision_osd.cpp
    src/ocr_pipeline.cpp src/road_speed_estimator.cpp src/ufld_lane_model.cpp
    src/yolo.cpp src/p3_road_safety_advisor.cpp src/experiment_metrics.cpp
    src/local_segments_mirror.cpp src/pci_bar0.cpp src/pcie_frame_source.cpp
    src/perf_config.cpp src/postprocess.c src/selector_mirror.cpp src/twin_route.cpp)
add_executable(adas_app ${APP_SOURCES})
target_include_directories(adas_app PRIVATE ${APP_INCLUDE})
target_link_libraries(adas_app PRIVATE PkgConfig::OPENCV PkgConfig::GTK Threads::Threads
    "${DEPLOY_LIB}/librknnrt.so" "${DEPLOY_LIB}/libhyperlpr3.so" "${DEPLOY_LIB}/librga.so.2")
target_link_options(adas_app PRIVATE
    "-Wl,--wrap=_ZN17TwinliteLaneModel4initERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE")
set_target_properties(adas_app PROPERTIES BUILD_RPATH "${DEPLOY_LIB}")

include(CTest)
if(BUILD_TESTING)
    add_executable(test_hw_osd tests/test_hw_osd.cpp)
    target_include_directories(test_hw_osd PRIVATE ${APP_INCLUDE})
    target_compile_options(test_hw_osd PRIVATE -UNDEBUG)
    add_test(NAME hw_osd COMMAND test_hw_osd)

    add_executable(test_crosswalk tests/test_crosswalk.cpp src/crosswalk_person.cpp)
    target_include_directories(test_crosswalk PRIVATE ${APP_INCLUDE})
    target_compile_options(test_crosswalk PRIVATE -UNDEBUG)
    target_link_libraries(test_crosswalk PRIVATE PkgConfig::OPENCV Threads::Threads)
    add_test(NAME crosswalk COMMAND test_crosswalk)

    foreach(test_name baseline_ad_request interaction_profile)
        add_executable(${test_name} tests/${test_name}.cpp src/p3_decision_bridge.cpp
            src/p3_road_safety_advisor.cpp src/pci_bar0.cpp src/perf_config.cpp)
        target_include_directories(${test_name} PRIVATE ${APP_INCLUDE} "${CMAKE_CURRENT_SOURCE_DIR}/tests")
        target_compile_definitions(${test_name} PRIVATE AD_ACK_HELPER)
        target_compile_options(${test_name} PRIVATE -O1 -UNDEBUG -ffunction-sections -fdata-sections)
        target_link_options(${test_name} PRIVATE -Wl,--gc-sections)
        target_link_libraries(${test_name} PRIVATE PkgConfig::OPENCV Threads::Threads)
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
    set_tests_properties(hw_osd crosswalk baseline_ad_request interaction_profile PROPERTIES TIMEOUT 30)
endif()
