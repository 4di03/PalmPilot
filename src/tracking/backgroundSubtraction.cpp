#include <opencv2/opencv.hpp>
#include <iostream>
#include "backgroundSubtraction.h"




int main() {
    // Open the default webcam
    cv::VideoCapture cap(0); // 0 is the ID for the default camera
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open the webcam." << std::endl;
        return -1;
    }
    cv::Mat background = initBackground();
    // get size of background image
    cv::Size size = background.size();
    cv::Mat frame, foreground;
    int frameCount = 0;

    std::cout << "Press 'q' to quit." << std::endl;

    while (true) {
        cap >> frame; // Capture a new frame
        if (frame.empty()) {
            std::cerr << "Error: Empty frame captured." << std::endl;
            break;
        }
        // Resize frame if necessary (optional)
        cv::resize(frame, frame, size);

        // Perform background subtraction
        foreground = backgroundSubtraction(background, frame);

        // Display the results
        cv::imshow("Webcam Stream", frame);
        cv::imshow("Foreground", foreground);

        // Break on 'q' key press
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    cap.release(); // Release the webcam
    cv::destroyAllWindows(); // Close all OpenCV windows
    return 0;
}