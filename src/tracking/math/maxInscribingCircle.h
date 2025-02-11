#include <opencv2/opencv.hpp>

struct Circle{
    cv::Point center;
    double radius;
};

// gets the max inscribing circle of a contour
Circle getMaxInscribingCircle(const std::vector<cv::Point>& contour, const cv::Mat& img);



