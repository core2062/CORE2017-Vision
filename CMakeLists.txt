cmake_minimum_required(VERSION 3.6)
project(CORE2017-Vision)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(COREVisionLib ${PROJECT_SOURCE_DIR}/COREVisionLib)

find_package(OpenCV COMPONENTS core imgproc highgui videoio)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${COREVisionLib}/3rdparty/cscore/include)
include_directories(${COREVisionLib}/3rdparty/NetworkTables/include)
include_directories(${COREVisionLib}/3rdparty/llvm/include)
include_directories(${COREVisionLib})

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    link_directories(${COREVisionLib}/3rdparty/cscore/Linux/arm-raspbian)
    link_directories(${COREVisionLib}/3rdparty/NetworkTables/Linux/arm-raspbian)
elseif(WIN32)
    link_directories(${COREVisionLib}/3rdparty/cscore/Windows/x86)
    link_directories(${COREVisionLib}/3rdparty/NetworkTables/Windows/x86)
else()
    link_directories(${COREVisionLib}/3rdparty/cscore/Linux/amd64)
    link_directories(${COREVisionLib}/3rdparty/NetworkTables/Linux/amd64)
endif()

set(SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/src/main.cpp
        ${COREVisionLib}/src/CORETimer.cpp)
add_executable(CORE2017-Vision ${SOURCE_FILES})

if(WIN32)
    target_link_libraries(CORE2017-Vision cscore.dll ntcore.dll ${OpenCV_LIBS})
else()
    target_link_libraries(CORE2017-Vision cscore.so ntcore.so ${OpenCV_LIBS})
endif()