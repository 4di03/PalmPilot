#include <opencv2/opencv.hpp>


class HandTracker{
    // Interface for strategies to track the hand in the image
    public:
    // Gets the keypoints of the hand in the image 
    virtual std::vector<std::vector<double>> getKeypoints(cv::Mat image) = 0;

};