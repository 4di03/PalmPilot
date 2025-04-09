#include <opencv2/opencv.hpp>
#pragma once



#define NULL_POINT cv::Point(-1,-1) // null value for a point
struct HandData{
    cv::Point indexFingerPosition; // point of the index finger if it and the thumb are raised, (-1,-1) if it is not raised
    int numFingersRaised; // number of fingers raised, between 0 and 5
    bool handDetected; // True if the hand contour is in frame, else False
};


// lower-level data used for hand tracking
struct HandTrackingState{
    std::vector<cv::Point>& handContour; // contour of the hand
}

struct HandDataOutput{
    HandData handData; // data about the hand (public for users)
    HandTrackingState trackingState; // state of the hand tracking (meant for recording state for future next frame)
};


class HandKeypointTracker{
    // Interface for strategies to track the hand in the image
    public:
    // Gets the keypoints of the hand in the image , will be a list of the keypoints. If the point cannot be found , it will be (-1,-1)
    virtual std::vector<cv::Point> getKeypoints(const cv::Mat& image) = 0;

};


class HandTracker{
    // Interface for strategies to track bare minimum hand data needed for computer control
    public:
    virtual HandDataOutput getHandData(const cv::Mat& image, const HandTrackingState& previousTrackingState) = 0;

};