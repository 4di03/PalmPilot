#include "cam.h"
#include "handTracker.h"
#include "handTracking.h"
#include "calibration.h"
#include <chrono>
#include <opencv2/core/ocl.hpp>
#include "constants.h"
#define INTERP_INTERVAL 1 // extracts the keypoints every 10 frames
#define ANONYMIZE true // blurs the image to anonymize the user
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


/**
 * Displays the hand data on the image
 * @param img the image to draw the keypoints on
 * @param handData handData info to print/display
 */
void displayHandData(cv::Mat& img, HandData& handData) {
    static TrackingRect trackingBox = parseTrackingBox(TRACKING_BOX_FILE);

    // Create a blurred version of the entire image

    cv::Mat trackingRegion = trackingBox.cropImage(img);

    if (ANONYMIZE){
    cv::Mat blurredImg;

    cv::GaussianBlur(img, img, cv::Size(21, 21), 1000);
    img *= 0.1;
    }

    // Overlay the unblurred tracking region back onto the blurred image
    trackingRegion.copyTo(img(cv::Rect(trackingBox.topLeft, trackingBox.bottomRight)));

    // Draw the tracking box
    trackingBox.draw(img);

    // Adjust index finger position relative to the tracking box
    if (handData.indexFingerPosition.x != -1) {
        cv::Point adjustedPoint = handData.indexFingerPosition + cv::Point(trackingBox.topLeft.x, trackingBox.topLeft.y);

        // Draw the index finger position
        cv::circle(img, adjustedPoint, 3, cv::Scalar(0, 200, 0), -1);
    }

    // Display the number of fingers raised
    cv::putText(img, std::to_string(handData.numFingersRaised), cv::Point(10, 50), 
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 200, 0), 2);

    // Display whether the hand is detected
    cv::putText(img, handData.handDetected ? "True" : "False", cv::Point(10, 100), 
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 200, 0), 2);
}
// runs hand tracker on vide stream and draws keypoints 
void plotHandKeypoints(HandKeypointTracker* tracker){
    std::cout << "Running plotHandKeypoints" << std::endl;
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

        if (frame.empty()) {
            std::cerr << "Error: Failed to capture frame." << std::endl;
            break;
        }

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
    // record start time
    auto firstStart = std::chrono::high_resolution_clock::now();

    int frameCount = 0;
    while (true) {
        ct++;
        frameCount++;
        cv::Mat frame = stream.getFrame();

        
        // Check if the frame is empty
        if (frame.empty()) {
            std::cerr << "Error: Failed to capture frame." << std::endl;
            break;
        }
        HandData handData = HandData{cv::Point(-1, -1), 0, false};
        HandTrackingState previousTrackingState = HandTrackingState{std::vector<cv::Point>(), 
            std::vector<cv::Point>(), 
            std::vector<ConvexityDefect>(), 
            cv::Point(-1,-1), 
            std::vector<int>(),
             TrackingRect(cv::Point(0,0),0,cv::Point(0,0))};
        if (ct % INTERP_INTERVAL == 0){

            auto start = std::chrono::high_resolution_clock::now();
            HandDataOutput handDataOutput = tracker->getHandData(frame, previousTrackingState);
            handData = handDataOutput.handData;
            previousTrackingState = handDataOutput.trackingState;

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed = end - start;

            if (DEBUG){
                std::cout << "getHandData Execution time: " << elapsed.count() << " ms\n";
            }

            ct = 0; // reset the counter to avoid overflow
        }

    

        // if the keypoints are found, draw them on the image, else use the previous keypoints
        // auto start = std::chrono::high_resolution_clock::now();
        //if (DEBUG){
        displayHandData(frame,handData);
        //}
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double, std::milli> elapsed = end - start;
        // std::cout << "drawKeypoints Execution time: " << elapsed.count() << " ms\n";


        cv::imshow("Webcam Stream with hands", frame);

        // print the frame rate every 100 frames
        if (frameCount % 100 == 0){
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed = end - firstStart;

            std::cout << "Frame Rate: " << 1000*frameCount/elapsed.count() << " fps" << std::endl;
            firstStart = std::chrono::high_resolution_clock::now();
            frameCount = 0;
        }

        // Exit the loop when 'q' is pressed
        if (cv::waitKey(1) == 'q') {
            break;
        }

    }

    // Release the camera and destroy all windows
    cv::destroyAllWindows();

    }
}
