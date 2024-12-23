#include <iostream>
#include "cam.h"

// prototoype for displaying the webcam stream
int main() {
    // Open the default camera (0 represents the default camera)
    VideoStream stream(0);

    // Create a window to display the video
    cv::namedWindow("Webcam Stream", cv::WINDOW_AUTOSIZE);

    while (true) {
        cv::Mat frame = stream.getFrame();

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
    cv::destroyAllWindows();

    return 0;
}
