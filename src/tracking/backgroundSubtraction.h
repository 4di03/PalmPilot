#include <opencv2/opencv.hpp>
#include "handTracking.h"
#define THRESH_MIN 0
#define THRESH_MAX 255
#define FOREGROUND_DILATION_ITERATIONS 10
cv::Mat initBackground(){
    std::cout << "initalizing background image." << std::endl;
    cv::Mat background = cv::imread("data/background.png");
    cv::resize(background, background, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
    return background;
}
// Perform background subtraction by subtracting the background from the frame, and retursn the foreground
cv::Mat backgroundSubtraction(const cv::Mat& background, const cv::Mat& frame) {
    // Ensure both images have the same size and type
    cv::Mat output;
    if (background.size() != frame.size() || background.type() != frame.type()) {
        std::cerr << "Background and frame must have the same size and type!" << std::endl;
        exit(1);
    }

    cv::Mat diff = frame - background;
    cv::Mat mask;

    cv::inRange(diff, THRESH_MIN, THRESH_MAX, mask); // 255 for values within the threshold, 0 otherwise

    // dilate the mask to fill in holes
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), FOREGROUND_DILATION_ITERATIONS);

    cv::bitwise_and(frame, frame, output, mask);

    return output;
}