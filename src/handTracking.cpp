#include "cam.h"
#include "handTracker.h"
#include "handTracking.h"
#include <chrono>
#include <opencv2/core/ocl.hpp>
#define INTERP_INTERVAL 10 // extracts the keypoints every 10 frames

/**
 * Draws the keypoints on the image
 * @param image the image to draw the keypoints on
 * @param keypoints the keypoints to draw (22,2) vector
 */
void drawKeypoints(cv::Mat& img, std::vector<cv::Point>& keypoints){

    for (int n = 0; n < keypoints.size() ; n++){
        // lookup 2 connected body/hand parts
        cv::Point2f a = keypoints[n];
        // we did not find enough confidence before
        if (a.x<=0 || a.y<=0)
            continue;

        circle(img, a, 3, cv::Scalar(0,0,200), -1);
    }
}
// runs hand tracker on vide stream and draws keypoints 
void runHandTracking(HandTracker* tracker){

    if (cv::ocl::haveOpenCL()) {
        std::cout << "OpenCL is available!" << std::endl;
        cv::ocl::setUseOpenCL(true);
    } else {
        std::cout << "OpenCL is not available on this system." << std::endl;
    }
    {
    VideoStream stream(0);
    int ct = 0;
    std::vector<cv::Point> keypoints = std::vector<cv::Point>(22,cv::Point(-1,-1));
    while (true) {
        ct++;
        cv::Mat frame = stream.getFrame();
        cv::resize(frame, frame, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));

        // Check if the frame is empty
        if (frame.empty()) {
            std::cerr << "Error: Failed to capture frame." << std::endl;
            break;
        }

        if (ct % INTERP_INTERVAL == 0){

            auto start = std::chrono::high_resolution_clock::now();
            keypoints = tracker->getKeypoints(frame);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed = end - start;
            //std::cout << "getKeypoints Execution time: " << elapsed.count() << " ms\n";


            ct = 0; // reset the counter to avoid overflow
        }

        // if the keypoints are found, draw them on the image, else use the previous keypoints
        // auto start = std::chrono::high_resolution_clock::now();
        drawKeypoints(frame,keypoints);
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double, std::milli> elapsed = end - start;
        // std::cout << "drawKeypoints Execution time: " << elapsed.count() << " ms\n";


        cv::imshow("Webcam Stream with hands", frame);
        // Exit the loop when 'q' is pressed
        if (cv::waitKey(1) == 'q') {
            break;
        }

    }

    // Release the camera and destroy all windows
    cv::destroyAllWindows();

    }
}

