#include <opencv2/opencv.hpp>
#include "cam.h"
#include <iostream>
#define TL_X 700    
#define TL_Y 300
#define SIZE 50

// Callback function for trackbar updates (does nothing but is required)
void on_trackbar(int, void*) {}

int main() {
    // Load the background image
    VideoStream stream(0);



    // Create a window
    const std::string windowName = "Calibration";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

    // Variables for YCrCb range
    int yMin = 0, yMax = 255;
    int crMin = 0, crMax = 255;
    int cbMin = 0, cbMax = 255;

    // Create trackbars
    cv::createTrackbar("Y Min", windowName, &yMin, 255, on_trackbar);
    cv::createTrackbar("Y Max", windowName, &yMax, 255, on_trackbar);
    cv::createTrackbar("Cr Min", windowName, &crMin, 255, on_trackbar);
    cv::createTrackbar("Cr Max", windowName, &crMax, 255, on_trackbar);
    cv::createTrackbar("Cb Min", windowName, &cbMin, 255, on_trackbar);
    cv::createTrackbar("Cb Max", windowName, &cbMax, 255, on_trackbar);

    // Calibration loop
    while (true) {

        // Convert the background image to YCrCb
        cv::Mat bgImageYCrCb;
        cv::cvtColor(stream.getFrame(), bgImageYCrCb, cv::COLOR_BGR2YCrCb);

        // draw a box and the average YCrCb values of it to help user calibrate
        cv::rectangle(bgImageYCrCb, cv::Point(TL_X, TL_Y), cv::Point(TL_X + SIZE, TL_Y + SIZE), cv::Scalar(0, 255, 0), 2);
        // get average YCrCb values of the box
        cv::Mat roi = bgImageYCrCb(cv::Rect(TL_X, TL_Y, SIZE, SIZE));
        cv::Scalar avg = cv::mean(roi);
        cv::putText(bgImageYCrCb, "Avg Y: " + std::to_string(avg[0]), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        cv::putText(bgImageYCrCb, "Avg Cr: " + std::to_string(avg[1]), cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        cv::putText(bgImageYCrCb, "Avg Cb: " + std::to_string(avg[2]), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

        // Show the mask
        cv::Mat mask;
        cv::inRange(bgImageYCrCb, cv::Scalar(yMin, crMin, cbMin), cv::Scalar(yMax, crMax, cbMax), mask);

        cv::imshow(windowName, mask);
        cv::imshow("Original", bgImageYCrCb);

        // Exit if the user presses 'q'
        char key = static_cast<char>(cv::waitKey(30)); // updates to slider variables are made here
        if (key == 'q' || key == 27) break; // 'q' or 'ESC' to quit
    }

    cv::destroyAllWindows();
    return 0;
}