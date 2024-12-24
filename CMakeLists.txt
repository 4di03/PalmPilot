# Minimum required version of CMake
cmake_minimum_required(VERSION 3.10)

# Project name
project(MouseControl)

# opencv setup
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# Set the output directory for the executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Add the source file
add_executable(mouseControl src/mouseControl.cpp) 
add_executable(webcam_stream src/cam.cpp )
add_executable(hand_tracking src/handTracker.cpp src/cam.h)
add_executable(openpose src/openpose.cpp)

# Link macOS frameworks
target_link_libraries(mouseControl PRIVATE "-framework CoreGraphics" "-framework CoreFoundation")
target_link_libraries(webcam_stream ${OpenCV_LIBS})
target_link_libraries(hand_tracking ${OpenCV_LIBS})
target_link_libraries(openpose ${OpenCV_LIBS})