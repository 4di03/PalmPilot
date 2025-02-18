# Minimum required version of CMake
cmake_minimum_required(VERSION 3.10)

# minimum required version of C++
set(CMAKE_CXX_STANDARD 14)


# Project name
project(PalmPilot)

# opencv setup
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Enables debug symbols, remove this line if you want to build a release version
set(CMAKE_BUILD_TYPE Debug) 


# Set the output directory for the executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})


# TODO: convert library header imports to <> instead of ""
#create a library for tracking
add_library(tracking src/tracking/fastTracker.cpp src/tracking/cam.h src/tracking/handTracking.cpp src/tracking/math/convexityDefects.cpp src/tracking/removeFace.cpp src/tracking/backgroundSubtraction.cpp src/tracking/math/kCurvature.cpp src/tracking/math/maxInscribingCircle.cpp src/tracking/calibration.cpp src/tracking/constants.h)

target_include_directories(tracking PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/tracking)
target_link_libraries(tracking ${OpenCV_LIBS})

# Add the source file
add_executable(mouseControl src/mouseControl/mouseControl.cpp) 
add_executable(webcam_stream src/tracking/cam.cpp )
add_executable(op_tracking src/tracking/openposeTracker.cpp)
add_executable(fast_tracking src/tracking/runFastTracking.cpp)
add_executable(openpose src/tracking/openposeSample.cpp)
add_executable(check_metal src/tracking/metalTest.cpp)
add_executable(calibration src/tracking/runCalibration.cpp)

# Link macOS frameworks
target_link_libraries(mouseControl PRIVATE "-framework CoreGraphics" "-framework CoreFoundation" tracking)
target_link_libraries(webcam_stream PRIVATE tracking)
target_link_libraries(op_tracking  PRIVATE tracking)
target_link_libraries(fast_tracking  PRIVATE tracking)
target_link_libraries(openpose PRIVATE tracking)
target_link_libraries(check_metal  PRIVATE tracking)
target_link_libraries(calibration  PRIVATE tracking)