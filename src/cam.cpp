#include <iostream>
#include <opencv2/opencv.hpp>


#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Open the default camera (0 represents the default camera)
    cv::VideoCapture cap(0);

    // Check if the camera opened successfully
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open webcam." << std::endl;
        return -1;
    }

    std::cout << "Press 'q' to exit the stream." << std::endl;

    // Create a window to display the video
    cv::namedWindow("Webcam Stream", cv::WINDOW_AUTOSIZE);

    while (true) {
        cv::Mat frame;

        // Capture each frame
        cap >> frame;

        // Check if the frame is empty
        if (frame.empty()) {
            std::cerr << "Error: Failed to capture frame." << std::endl;
            break;
        }

        // Display the frame
        cv::imshow("Webcam Stream", frame);

        // Exit the loop when 'q' is pressed
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Release the camera and destroy all windows
    cap.release();
    cv::destroyAllWindows();

    return 0;
}
