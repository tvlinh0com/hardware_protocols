cmake_minimum_required(VERSION 3.23)
project(HWPLibrary VERSION 1.0.0 LANGUAGES CXX)

option(HWPLIB_BUILD_EXAMPLE "Build examples using Hardware Protocols Library" OFF)

if(HWPLIB_BUILD_EXAMPLE)
    add_subdirectory(examples)
endif()

add_library(hwp SHARED
    src/hardware_protocols/i2c/i2c_ch347.cpp
)

target_sources(hwp PUBLIC
    FILE_SET HEADERS
    BASE_DIRS include
    FILES include/hardware_protocols/i2c/i2c.hpp include/hardware_protocols/i2c/i2c_ch347.hpp
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0)

target_link_libraries(hwp PUBLIC PkgConfig::libusb)

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
