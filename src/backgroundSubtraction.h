#include <opencv2/opencv.hpp>
#include "handTracking.h"
#define THRESH_MIN 50
#define THRESH_MAX 255
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

    // Split the channels
    std::vector<cv::Mat> bgChannels, frameChannels;
    cv::split(background, bgChannels);
    cv::split(frame, frameChannels);

    // Initialize containers for processed channels
    std::vector<cv::Mat> processedChannels(bgChannels.size());

    // Process each channel independently
    for (size_t i = 0; i < bgChannels.size(); ++i) {
        // Absolute difference for the current channel
        cv::Mat diff;
        cv::absdiff(bgChannels[i], frameChannels[i], diff);

        // Normalize the difference to the range [0, 255]
        double minVal, maxVal;
        cv::minMaxLoc(diff, &minVal, &maxVal);
        if (maxVal > 0) {
            diff.convertTo(diff, CV_8U, 255.0 / maxVal);
        }

        // Threshold dynamically
        cv::Mat thresh;
        double threshMin = THRESH_MIN; // Minimum difference to consider significant
        double threshMax = THRESH_MAX; // Maximum value (already normalized)
        cv::inRange(diff, threshMin, threshMax, thresh);

        // Apply morphological operations to reduce noise
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(thresh, processedChannels[i], cv::MORPH_OPEN, kernel);
    }

    // Merge the channels using addition
    cv::Mat mergedChannels = cv::Mat::zeros(processedChannels[0].size(), processedChannels[0].type());
    for (const auto& channel : processedChannels) {
        cv::add(mergedChannels, channel, mergedChannels);
    }

    // Mask the original frame
    cv::Mat mask;
    cv::threshold(mergedChannels, mask, 50, 255, cv::THRESH_BINARY);
    cv::bitwise_and(frame, frame, output, mask);

    return output;
}