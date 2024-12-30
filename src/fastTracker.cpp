#include "handTracking.h"
#include "handTracker.h"
#include "removeFace.h"
#include <opencv2/opencv.hpp>
#include <opencv2/bgsegm.hpp>

#define MAX_DIST 30
#define INTERVAL 30 // interval for definiting the skin color range givne an image of the palm
#define PALM_IMAGE_PATH "data/palm.png"
#define BLUR_SIZE 10
#define MIN_SOLIDITY 0.5
#define MAX_SOLIDITY 0.8
#define MIN_PROP 0.01 // minimum proportion of the image that the hand contour can take up to be considered valid
#define MAX_PROP 0.3 // maximum proportion of the image that the hand contour can take up to be considered valid

struct colorRange{
    cv::Scalar lower;
    cv::Scalar upper;
};
// gets a single contour from a list of contours
class ContourFilterStrategy{
    public:
        virtual std::vector<cv::Point> filterContour(std::vector<std::vector<cv::Point>> contours) = 0;
};
// gets valid contours
class ValidContourStrategy{
    public:
        virtual std::vector<std::vector<cv::Point>> getValidContours(std::vector<std::vector<cv::Point>> contours) = 0;
};

// gets contour with largest area
class MaxAreaFilter : public ContourFilterStrategy{
    public:
        std::vector<cv::Point> filterContour(std::vector<std::vector<cv::Point>> contours){
            if (contours.empty()) {
                return std::vector<cv::Point>();
            }
            std::vector<cv::Point> maxContour = *(std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& c0, const std::vector<cv::Point>& c1) {
                return cv::contourArea(c0) < cv::contourArea(c1);
            }));
            return maxContour;
        }
};
// gets contouor with ratio of contour area to convex hull area that is between a certain range, if many exist in that range it, then it applies a secondary filter to get the best one
class SolidityFilter : public ValidContourStrategy{

    public:
        std::vector<std::vector<cv::Point>> getValidContours(std::vector<std::vector<cv::Point>> contours){
            if (contours.empty()) {
                return std::vector<std::vector<cv::Point>>();
            }
            // Sort contours by area in descending order and return the largest one

            std::vector<std::vector<cv::Point>> validContours;
            for (auto contour : contours){
                std::vector<cv::Point> hull;
                cv::convexHull(contour, hull);
                double area = cv::contourArea(contour);
                double hullArea = cv::contourArea(hull);
                double solidity = area/hullArea;
                if (solidity > MIN_SOLIDITY && solidity < MAX_SOLIDITY){
                    validContours.push_back(contour);
                }
            }
            return validContours;
        }
};
// filters out contours that take up too much or too little of the image
class AreaFilter : public ValidContourStrategy{
    public:
        std::vector<std::vector<cv::Point>> getValidContours(std::vector<std::vector<cv::Point>> contours){
            std:: cout  << "Applying area filter" << std::endl;
            if (contours.empty()) {
                return std::vector<std::vector<cv::Point>>();
            }
            // Sort contours by area in descending order and return the largest one

            std::vector<std::vector<cv::Point>> validContours;
            double imageArea = FRAME_WIDTH * FRAME_HEIGHT;
            for (auto contour : contours){
                double prop = cv::contourArea(contour) / imageArea;
            
                if (prop > MIN_PROP && prop < MAX_PROP){

                    validContours.push_back(contour);
                }
            }
            return validContours;
        }

};

class CompositeFilter : public ContourFilterStrategy{
    public:
        std::vector<ValidContourStrategy*> validFilters;
        ContourFilterStrategy* finalSelector;
        CompositeFilter(std::vector<ValidContourStrategy*> validFilters, ContourFilterStrategy* finalSelector){
            this->validFilters = validFilters;
            this->finalSelector = finalSelector;
        }
        std::vector<cv::Point> filterContour(std::vector<std::vector<cv::Point>> contours){
            std::vector<std::vector<cv::Point>> validContours(contours);

            // Apply each filter in sequence until a valid contour is found
            for (auto filter : validFilters){
                validContours = filter->getValidContours(validContours);
                if (!validContours.empty()){
                    break;
                }
            }

            // apply the final filter
            return finalSelector->filterContour(validContours);
        }
};

 

// gets a lower and upper range of the HLS colors given the average color of the palm
colorRange getRangeFromImage(std::string imagePath){
    cv::Mat image = cv::imread(imagePath);
    cv::Mat imgHLS;
    cv::cvtColor(image, imgHLS, cv::COLOR_BGR2HLS);
    cv::Scalar avgHLS = cv::mean(imgHLS);
    cv::Scalar lower = cv::Scalar(avgHLS[0] - INTERVAL, avgHLS[1] - INTERVAL, avgHLS[2] - INTERVAL);
    cv::Scalar upper = cv::Scalar(avgHLS[0] + INTERVAL, avgHLS[1] + INTERVAL, avgHLS[2] + INTERVAL);

    return colorRange{lower,upper};
}
/**
 * Applies a binary mask to an image.
 * @param image The input image (original frame).
 * @param mask The binary mask (single channel).
 * @return The masked image with the background removed.
 */
cv::Mat applyMask(const cv::Mat& image, const cv::Mat& mask) {
    cv::Mat result;

    // Ensure the mask has a single channel
    if (mask.channels() != 1) {
        throw std::invalid_argument("Mask must be a single-channel binary image");
    }

    // Ensure the mask and image have the same size
    if (image.size() != mask.size()) {
        throw std::invalid_argument("Image and mask sizes must match");
    }

    // Apply the mask to the image
    cv::bitwise_and(image, image, result, mask);

    return result;
}
// Segmenting by skin color
auto skinColorUpper = [](int hue) { return cv::Scalar(hue, 0.5 * 255, 0.6 * 255); };
auto skinColorLower = [](int hue) { return cv::Scalar(hue, 0.1 * 255, 0.05 * 255); };


cv::Mat makeHandMask(const cv::Mat& image){
    // Filter by skin color
    cv::Mat img = removeFace(image);

    


    cv::cvtColor(img, img, cv::COLOR_BGR2HLS);




    cv::Mat rangeMask;
    colorRange range = getRangeFromImage(PALM_IMAGE_PATH);



    cv::inRange(img, range.lower, range.upper, rangeMask);
    // dilation to fill gaps in the hand mask
    cv::dilate(rangeMask, rangeMask, cv::Mat(), cv::Point(-1, -1), 4); 

    // closing operation to fill small holes inside the foreground
    cv::morphologyEx(rangeMask, rangeMask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 4); 

    // Gaussian blur to smooth the mask
    cv::GaussianBlur(rangeMask, rangeMask, cv::Size(5, 5), 0); 

    return rangeMask;
}

// Function to calculate the distance between two points
double ptDist(const cv::Point& pt1, const cv::Point& pt2) {
    return cv::norm(pt1 - pt2);
}

// Function to get the center point of a group of points
cv::Point getCenterPt(const std::vector<cv::Point>& points) {
    cv::Point center(0, 0);
    for (const auto& pt : points) {
        center += pt;
    }
    center.x /= points.size();
    center.y /= points.size();
    return center;
}

// Function to get the rough hull
// Returns the contour indices for the most central points
std::vector<int> getRoughHull(const std::vector<cv::Point>& contour, double maxDist) {
    
    // Get hull indices and hull points
    std::vector<int> hullIndices;
    if (contour.empty()) {
    std::cerr << "Error: Input to convexHull is empty." << std::endl;
    return hullIndices;
    }

    cv::convexHull(contour, hullIndices, false, false);
    std::vector<cv::Point> hullPoints;
    for (int idx : hullIndices) {
        hullPoints.push_back(contour[idx]);
    }

    // Group all points in local neighborhood
    auto ptsBelongToSameCluster = [&](const cv::Point& pt1, const cv::Point& pt2) {
        return ptDist(pt1, pt2) < maxDist;
    };
    std::vector<int> labels;
    cv::partition(hullPoints, labels, ptsBelongToSameCluster);

    std::map<int, std::vector<std::pair<cv::Point, int>>> pointsByLabel;
    for (size_t i = 0; i < labels.size(); ++i) {
        pointsByLabel[labels[i]].emplace_back(hullPoints[i], hullIndices[i]);
    }

    // Map points in local neighborhood to most central point
    auto getMostCentralPoint = [&](const std::vector<std::pair<cv::Point, int>>& pointGroup) {
        // Find center
        std::vector<cv::Point> points;
        for (const auto& ptWithIdx : pointGroup) {
            points.push_back(ptWithIdx.first);
        }
        cv::Point center = getCenterPt(points);

        // Sort ascending by distance to center
        auto compare = [&](const std::pair<cv::Point, int>& ptWithIdx1, const std::pair<cv::Point, int>& ptWithIdx2) {
            return ptDist(ptWithIdx1.first, center) < ptDist(ptWithIdx2.first, center);
        };
        return *std::min_element(pointGroup.begin(), pointGroup.end(), compare);
    };

    std::vector<int> result;
    for (const auto& pointGroup : pointsByLabel) {
        result.push_back(getMostCentralPoint(pointGroup.second).second);
    }

    return result;
}


// Gets the keypoints of the fingertips of the hand using Contour Detection and Convex Hull
class FastTracker : public HandTracker {
    public:
        ContourFilterStrategy* filterStrategy;
        FastTracker(ContourFilterStrategy* filterStrategy){ 
            this->filterStrategy = filterStrategy;

        }

        std::vector<cv::Point> getKeypoints(const cv::Mat& image){

            cv::Mat handMask = makeHandMask(image);
                
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(handMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            std::vector<cv::Point> contour = filterStrategy->filterContour(contours);
            
            // draw the contour
            cv::Mat contourImage = cv::Mat::zeros(handMask.size(), CV_8UC3);
            cv::drawContours(contourImage, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
            cv::imshow("Contour", contourImage);

            std::vector<int> fingertipIndices = getRoughHull(contour, MAX_DIST);
            std::vector<cv::Point> fingertipPoints = {};
            for (int idx : fingertipIndices) {
                fingertipPoints.push_back(contour[idx]);
            }
            cv::imshow("Hand Mask", handMask);

            // TODO: standardize the shape of the output to handle things that may retuirn more or less than all the hand keypoints
            return fingertipPoints;
        }
};

// prototype for deriving keypoints from the webcame stream frames
int main(){

    ContourFilterStrategy* filter = new CompositeFilter({new AreaFilter(),new SolidityFilter()}, new MaxAreaFilter());
    HandTracker* tracker = new FastTracker(filter);

    runHandTracking(tracker);
    return 0;
}