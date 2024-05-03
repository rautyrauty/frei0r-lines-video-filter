cmake_minimum_required (VERSION 3.1)

project( frei0r-lines-video-filter
    VERSION 0.1.0
    LANGUAGES CXX
)

# Set path to additional CMake modules
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
find_package(frei0r REQUIRED)

add_subdirectory(src)
