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


set (FAST_TRACKING_SRC src/tracking/fastTracker.cpp src/tracking/cam.h src/tracking/handTracking.cpp src/tracking/removeFace.cpp src/tracking/backgroundSubtraction.h src/tracking/math/kCurvature.h src/tracking/math/maxInscribingCircle.cpp src/tracking/calibration.cpp src/tracking/constants.h)

# Add the source file
add_executable(mouseControl src/tracking/mouseControl.cpp) 
add_executable(webcam_stream src/tracking/cam.cpp )
add_executable(op_tracking src/tracking/openposeTracker.cpp src/tracking/cam.h src/tracking/handTracking.cpp)
add_executable(fast_tracking ${FAST_TRACKING_SRC})
add_executable(background_sub src/tracking/backgroundSubtraction.cpp )
add_executable(openpose src/tracking/openposeSample.cpp)
add_executable(check_metal src/tracking/metalTest.cpp)
add_executable(calibration src/tracking/runCalibration.cpp src/tracking/calibration.cpp)

# Link macOS frameworks
target_link_libraries(mouseControl PRIVATE "-framework CoreGraphics" "-framework CoreFoundation")
target_link_libraries(webcam_stream ${OpenCV_LIBS})
target_link_libraries(op_tracking ${OpenCV_LIBS})
target_link_libraries(fast_tracking ${OpenCV_LIBS})
target_link_libraries(openpose ${OpenCV_LIBS})
target_link_libraries(check_metal ${OpenCV_LIBS})
target_link_libraries(background_sub ${OpenCV_LIBS})
target_link_libraries(calibration ${OpenCV_LIBS})