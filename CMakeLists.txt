cmake_minimum_required(VERSION 3.23)
project(OLED_Driver)

option(OLED_BUILD_EXECUTABLE "Build OLED Controller executable" ON)

if(OLED_BUILD_EXECUTABLE)
    add_subdirectory(OLED_Main)
endif()

add_subdirectory(OLED_Lib)
