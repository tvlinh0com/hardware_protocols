cmake_minimum_required(VERSION 3.23)

if(NOT CMAKE_PROJECT_NAME STREQUAL "HWPLibrary")
    message(FATAL_ERROR "This example directory must be built as a subdirectory of the main project")
endif()

# Collect all example source files
file(GLOB EXAMPLE_SOURCES CONFIGURE_DEPENDS "*.cpp")

# Iterate through each source file found
foreach(SOURCE_FILE ${EXAMPLE_SOURCES})
    # Extract the filename without path or extension
    get_filename_component(EXAMPLE_NAME ${SOURCE_FILE} NAME_WE)
    # Define an executable matching the file name
    add_executable(${EXAMPLE_NAME} ${SOURCE_FILE})
    # Link against HWPLibrary
    target_link_libraries(${EXAMPLE_NAME} PRIVATE hwp)
    # Put example executable into same binary directory
    set_target_properties(${EXAMPLE_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
endforeach()
