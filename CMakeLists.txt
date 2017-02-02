cmake_minimum_required(VERSION 3.6)
project(COREVisionLib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lpthread")

find_package(OpenCV COMPONENTS core imgproc highgui videoio)
include_directories(${OpenCV_INCLUDE_DIRS})

set(SOURCE_FILES src/main.cpp)
add_executable(COREVisionLib ${SOURCE_FILES})

target_link_libraries(COREVisionLib ${OpenCV_LIBS})