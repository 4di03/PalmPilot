#include "maxInscribingCircle.h"


Circle getMaxInscribingCircle(const std::vector<cv::Point>& contour, const cv::Mat& img) {

    cv::Mat filledContour = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::drawContours(filledContour, {contour}, 0, cv::Scalar(255), cv::FILLED);

    // Get the min distance to a 0 pixel for each point in the filled contour
    cv::Mat distanceTransform;
    cv::distanceTransform(filledContour, distanceTransform, cv::DIST_L2, 3);
    
    
    // Find the maximum value and its location (guaranteed to be inside the contour)
    double maxVal;
    cv::Point maxLoc;
    minMaxLoc(distanceTransform, nullptr, &maxVal, nullptr, &maxLoc);

    return Circle{maxLoc, maxVal};

}