#include "handTracking.h"
#include "handTracker.h"
#include "removeFace.h"
#include "backgroundSubtraction.h"
#include "kCurvature.h"
#include "calibration.h"
#include <opencv2/opencv.hpp>
#include <opencv2/bgsegm.hpp>
#define MAX_DIST 30
#define INTERVAL 20 // interval for definiting the skin color range givne an image of the palm
#define ST_DEVS_DIFF 5 // number of standard deviations to consider for the range
#define COLOR_RANGE_FILE "data/color_range.yaml"
#define BLUR_SIZE 10
#define MIN_SOLIDITY 0.5
#define MAX_SOLIDITY 0.8
#define MIN_PROP 0.01 // minimum proportion of the image that the hand contour can take up to be considered valid
#define MAX_PROP 0.3 // maximum proportion of the image that the hand contour can take up to be considered valid
#define COLOR_CONVERSION cv::COLOR_BGR2YCrCb
#define MIN_CURVATURE 10 // min angle between hull vectors to be considered a fingertip
#define MAX_CURVATURE 60 // max angle between hull vectors to be considered a fingertip
#define DEBUG true
struct colorRange{
    cv::Scalar lower;
    cv::Scalar upper;
};


void printMat(const cv::Mat& mat) {
    for (int i = 0; i < mat.rows; i++) {
        for (int j = 0; j < mat.cols; j++) {
            // For single-channel matrices
            if (mat.channels() == 1) {
                std::cout << mat.at<uchar>(i, j) << " ";
            }
            // For multi-channel matrices 
            else if (mat.channels() == 3) {
                cv::Vec3b pixel = mat.at<cv::Vec3b>(i, j);
                std::cout << "[" << (int)pixel[0] << ", " << (int)pixel[1] << ", " << (int)pixel[2] << "] ";
            }
        }
        std::cout << std::endl;
    }
}
// gets a lower and upper range of the  colors given the average color of the palm
colorRange getRangeFromImage(std::string imagePath){
    cv::Mat image = cv::imread(imagePath);
    cv::Mat img;
    cv::cvtColor(image, img, COLOR_CONVERSION);
    cv::Scalar mean, stddev;
    cv::meanStdDev(img, mean, stddev);
    
    //printMat(img);


    // std:: cout << "Mean: " << mean << std::endl;
    // std:: cout << "Stddev: " << stddev << std::endl;
    cv::Scalar lower(mean[0] - ST_DEVS_DIFF * stddev[0] , mean[1] - ST_DEVS_DIFF * stddev[1], mean[2] - ST_DEVS_DIFF * stddev[2]);
    cv::Scalar upper(mean[0]  + ST_DEVS_DIFF * stddev[0], mean[1] + ST_DEVS_DIFF * stddev[1], mean[2] + ST_DEVS_DIFF * stddev[2]);
    
    // std::cout << "Lower: " << lower << std::endl;
    // std::cout << "Upper: " << upper << std::endl;
    return colorRange{lower,upper};
}


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

class HandMaskPostProcessingStrategy{
    public:
        virtual cv::Mat postProcess(cv::Mat& mask) = 0;
};

class GaussianBlurPostProcessing : public HandMaskPostProcessingStrategy{
    private:
        int blurSize;
    public:
        GaussianBlurPostProcessing(int blurSize = 5){
            this->blurSize = blurSize;
        }
        
        cv::Mat postProcess(cv::Mat& mask){
            cv::GaussianBlur(mask, mask, cv::Size(blurSize, blurSize), 0); 
            return mask;
        }
};

class DilationPostProcessing : public HandMaskPostProcessingStrategy{
    private:
        int dilationIterations;
    public:
        DilationPostProcessing(int dilationIterations = 4){
            this->dilationIterations = dilationIterations;
        }
        
        cv::Mat postProcess(cv::Mat& mask){
            cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), dilationIterations); 
            return mask;
        }
};

class ClosingPostProcessing : public HandMaskPostProcessingStrategy{
    private:
        int closingIterations;
    public:
        ClosingPostProcessing(int closingIterations = 4){
            this->closingIterations = closingIterations;
        }
        
        cv::Mat postProcess(cv::Mat& mask){
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), closingIterations); 
            return mask;
        }
};

class OpeningPostProcessing: public HandMaskPostProcessingStrategy{
    private:
        int openingIterations;
    public:
        OpeningPostProcessing(int openingIterations = 4){
            this->openingIterations = openingIterations;
        }
        
        cv::Mat postProcess(cv::Mat& mask){
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), openingIterations); 
            return mask;
        }
};
// applies a series of post processing strategies to the mask
class CompositePostProcessing : public HandMaskPostProcessingStrategy{
    private:
        // vector of post processing strategies, will be applied in order
        std::vector<HandMaskPostProcessingStrategy*> postProcessingStrategies;
    public:
        CompositePostProcessing(std::vector<HandMaskPostProcessingStrategy*> postProcessingStrategies){
            this->postProcessingStrategies = postProcessingStrategies;
        }
        
        cv::Mat postProcess(cv::Mat& mask){
            for (auto strategy : postProcessingStrategies){
                mask = strategy->postProcess(mask);
            }
            return mask;
        }
};

class ErosionPostProcessing : public HandMaskPostProcessingStrategy{
    private:
        int erosionIterations;
    public:
        ErosionPostProcessing(int erosionIterations = 4){
            this->erosionIterations = erosionIterations;
        }
        
        cv::Mat postProcess(cv::Mat& mask){
            cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), erosionIterations); 
            return mask;
        }
};
// gets binary mask of the hand
class HandMaskStrategy{
    private: 
        HandMaskPostProcessingStrategy* postProcessingStrategy;

    public:
        HandMaskStrategy(HandMaskPostProcessingStrategy* postProcessingStrategy){
            this->postProcessingStrategy = postProcessingStrategy;
        }
       
       cv::Mat makeHandMask(const cv::Mat& image){
            static colorRange range = parseColorRange(COLOR_RANGE_FILE);

            // Filter by skin color
            cv::Mat img;
            cv::cvtColor(image, img, COLOR_CONVERSION);

            cv::Mat rangeMask;

            cv::inRange(img, range.lower, range.upper, rangeMask);
            if (DEBUG){
            cv::imshow("Raw Mask", rangeMask);
            }

            return postProcessingStrategy->postProcess(rangeMask);;
        };
};


 


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

/**
 * Gets the keypoints of the fingertips of the hand usingConvex Hull and K-curvature
 */
std::vector<cv::Point> getFingertipPoints(const std::vector<cv::Point>& contour, const cv::Mat& img = cv::Mat()) {

    std::vector<int> fingertipIndices = getRoughHull(contour, MAX_DIST);
    std::vector<KCurvatureData> kCurvatures = getKCurvatureData(contour, fingertipIndices);
    
    if (DEBUG){
        cv::Mat KCurvatureImage = img.clone();
        for (KCurvatureData kCurvature : kCurvatures){
            cv::line(KCurvatureImage, kCurvature.start, kCurvature.point, cv::Scalar(0, 255, 0), 2);
            cv::line(KCurvatureImage, kCurvature.end, kCurvature.point, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("K Curvature", KCurvatureImage);
    }

    std::vector<cv::Point> fingertipPoints;

    for (KCurvatureData kCurvature : kCurvatures){
        double angle = kCurvature.getKCurvature();
        if (angle > MIN_CURVATURE && angle < MAX_CURVATURE){
            fingertipPoints.push_back(kCurvature.point);
        }
    }

    if (DEBUG){
        cv::Mat fingertipImage = img.clone();
        drawKeypoints(fingertipImage, fingertipPoints);
        cv::imshow("Fingertips", fingertipImage);
    }

    return fingertipPoints;
}

// Gets the keypoints of the fingertips of the hand using Contour Detection and Convex Hull
class FastTracker : public HandTracker {
    private:
        ContourFilterStrategy* filterStrategy;
        HandMaskStrategy* maskStrategy;
    public:
        FastTracker(ContourFilterStrategy* filterStrategy, HandMaskStrategy* maskStrategy){ 
            this->filterStrategy = filterStrategy;
            this->maskStrategy = maskStrategy;

        }

        HandData getHandData(const cv::Mat& image){
            static cv::Mat background = initBackground();

            cv::Mat foreground = backgroundSubtraction(background,(image));
            cv::Mat handMask = maskStrategy->makeHandMask(foreground);
            
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(handMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            std::vector<cv::Point> contour = filterStrategy->filterContour(contours);
            
            // draw the contour
            cv::Mat contourImage = cv::Mat::zeros(handMask.size(), CV_8UC3);
            cv::drawContours(contourImage, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);

            std::vector<cv::Point> fingertipPoints = getFingertipPoints(contour, image);
            if (DEBUG){
                cv::imshow("Foreground", foreground);
                cv::imshow("Contour", contourImage);
                cv::imshow("Hand Mask", handMask);
            }

            int numFingersRaised = static_cast<int>(fingertipPoints.size());
            // TODO: standardize the shape of the output to handle things that may retuirn more or less than all the hand keypoints
            return HandData{numFingersRaised > 0 ? fingertipPoints[0] : cv::Point(-1,-1),numFingersRaised,numFingersRaised > 0};
        }
};

// prototype for deriving keypoints from the webcame stream frames
int main(){

    HandMaskStrategy* maskStrategy = new HandMaskStrategy(
        new CompositePostProcessing(
            {
            new DilationPostProcessing(3), 
            new ErosionPostProcessing(10),
            new DilationPostProcessing(8),
            new GaussianBlurPostProcessing(1), 
            }
        )
    );
    ContourFilterStrategy* filter = new CompositeFilter({new AreaFilter(),new SolidityFilter()}, new MaxAreaFilter());
    HandTracker* tracker = new FastTracker(filter, maskStrategy);

    runHandTracking(tracker);
    return 0;
}