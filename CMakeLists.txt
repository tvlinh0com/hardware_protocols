cmake_minimum_required(VERSION 3.23)
project(HWPLibrary VERSION 1.0.0 LANGUAGES CXX)

option(HWPLIB_BUILD_EXAMPLE "Build examples using Hardware Protocols Library" OFF)

if(HWPLIB_BUILD_EXAMPLE)
    add_subdirectory(examples)
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_library(hwp SHARED
    src/hardware_protocols/i2c/i2c_ch347t.cpp
)

target_sources(hwp PUBLIC
    FILE_SET HEADERS
    BASE_DIRS include
    FILES include/hardware_protocols/i2c/i2c.hpp include/hardware_protocols/i2c/i2c_ch347t.hpp
)

find_path(LIBUSB_INCLUDE_DIR
    NAMES libusb.h
    PATH_SUFFIXES "libusb-1.0"
)

find_library(LIBUSB_LIBRARY
    NAMES usb-1.0 libusb-1.0 libusb
)

if(LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARY)
    message(STATUS "Found libusb library and its headers")
    target_include_directories(hwp PUBLIC ${LIBUSB_INCLUDE_DIR})
    target_link_libraries(hwp PUBLIC ${LIBUSB_LIBRARY})
else()
    message(FATAL_ERROR "FATAL ERROR: Could not find libusb library or headers!")
endif()

include(GNUInstallDirs)

install(TARGETS hwp
    EXPORT HWPLibraryTarget
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT HWPLibraryTarget
    FILE HWPLibraryTarget.cmake
    NAMESPACE HWPLibrary::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/HWPLibrary
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/HWPLibraryConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "cmake/HWPLibraryConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/HWPLibraryConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/HWPLibrary
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/HWPLibraryConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/HWPLibraryConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/HWPLibrary
)
