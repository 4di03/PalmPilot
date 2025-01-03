#include <opencv2/opencv.hpp>
#include "detectionData.h"
#pragma once

class HandKeypointTracker{
    // Interface for strategies to track the hand in the image
    public:
    // Gets the keypoints of the hand in the image , will be a list of the keypoints. If the point cannot be found , it will be (-1,-1)
    virtual std::vector<cv::Point> getKeypoints(const cv::Mat& image) = 0;

};


class HandTracker{
    // Interface for strategies to track bare minimum hand data needed for computer control
    public:
    virtual HandData getHandData(const cv::Mat& image) = 0;

};