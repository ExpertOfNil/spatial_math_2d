cmake_minimum_required(VERSION 3.16)
project(spatial_math)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)

set(
    CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs/
)
set(
    CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/
)

find_package(OpenCV 4.2 REQUIRED)

include_directories(
    src
	include
	/usr/include/
    ${CMAKE_SOURCE_DIR}/raylib/raylib-5.0_linux_amd64/include
)

#file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/main.cpp")

link_directories(${CMAKE_SOURCE_DIR}/raylib/raylib-5.0_linux_amd64/lib)

# Create static lib for transform
add_library(transform STATIC "src/transform.cpp")
target_link_libraries(transform
    ${OpenCV_LIBS}
    pthread
)

# Create executable and link libraries
add_executable(${PROJECT_NAME}
  "src/main.cpp"
)
target_link_libraries(${PROJECT_NAME}
    transform
    ${OpenCV_LIBS}
    ${LIBS}
    pthread
    dl
    m
    raylib
)
