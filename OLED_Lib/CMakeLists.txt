add_library(OLED_Library)
target_sources(OLED_Library
    PRIVATE src/OLED_library.cpp
    PUBLIC FILE_SET HEADERS BASE_DIRS headers FILES headers/OLED_library.h)
