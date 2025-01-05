# Minimum required version of CMake
cmake_minimum_required(VERSION 3.10)

# Project name
project(MouseControl)

# opencv setup
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Enables debug symbols, remove this line if you want to build a release version
set(CMAKE_BUILD_TYPE Debug) 


# Set the output directory for the executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Add the source file
add_executable(mouseControl src/mouseControl.cpp) 
add_executable(webcam_stream src/cam.cpp )
add_executable(op_tracking src/openposeTracker.cpp src/cam.h src/handTracking.cpp)
add_executable(fast_tracking src/fastTracker.cpp src/cam.h src/handTracking.cpp src/removeFace.cpp src/backgroundSubtraction.h src/kCurvature.h)
add_executable(background_sub src/backgroundSubtraction.cpp )
add_executable(openpose src/openposeSample.cpp)
add_executable(check_metal src/metalTest.cpp)
add_executable(calibration src/runCalibration.cpp src/calibration.cpp)

# Link macOS frameworks
target_link_libraries(mouseControl PRIVATE "-framework CoreGraphics" "-framework CoreFoundation")
target_link_libraries(webcam_stream ${OpenCV_LIBS})
target_link_libraries(op_tracking ${OpenCV_LIBS})
target_link_libraries(fast_tracking ${OpenCV_LIBS})
target_link_libraries(openpose ${OpenCV_LIBS})
target_link_libraries(check_metal ${OpenCV_LIBS})
target_link_libraries(background_sub ${OpenCV_LIBS})
target_link_libraries(calibration ${OpenCV_LIBS})