#include <opencv2/opencv.hpp>


class openposeTracker{
    // Interface for strategies to track the hand in the image
    public:
    // Gets the keypoints of the hand in the image , will be a list of the keypoints. If the point cannot be found , it will be (-1,-1)
    virtual std::vector<cv::Point> getKeypoints(cv::Mat image) = 0;

};