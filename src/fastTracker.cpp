#include "handTracking.h"
#include "handTracker.h"
#include <opencv2/opencv.hpp>
#include <opencv2/bgsegm.hpp>

#define MAX_DIST 30
#define INTERVAL 40 // interval for definiting the skin color range givne an image of the palm
#define PALM_IMAGE_PATH "data/palm.png"
#define BLUR_SIZE 10
struct colorRange{
    cv::Scalar lower;
    cv::Scalar upper;
};

/**
 * Subtracts the background from the given frame .
 * Initializes the background subtractor internally.
 * @param frame The current frame (input image).
 * @param backgroundSubtractor The background subtractor.
 * @return The foreground mask with the background subtracted.
 */
cv::Mat subtractBackground(const cv::Mat& frame, cv::Ptr<cv::BackgroundSubtractor> backgroundSubtractor) {

    cv::Mat fgMask;

    // Apply the background subtractor to the frame
    backgroundSubtractor->apply(frame, fgMask);

    
    // Optional: Clean up the foreground mask
    cv::erode(fgMask, fgMask, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(fgMask, fgMask, cv::Mat(), cv::Point(-1, -1), 1);

    return fgMask;
}
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

// Function to get the largest hand contour
std::vector<cv::Point> getHandContour(const cv::Mat& handMask) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(handMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Sort contours by area in descending order and return the largest one
    std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& c0, const std::vector<cv::Point>& c1) {
        return cv::contourArea(c0) > cv::contourArea(c1);
    });

    return contours.empty() ? std::vector<cv::Point>() : contours[0];
}
cv::Mat makeHandMask(const cv::Mat& image){
    // Filter by skin color
    cv::Mat img;
    cv::cvtColor(image, img, cv::COLOR_BGR2HLS);
    cv::Mat rangeMask;
    colorRange range = getRangeFromImage(PALM_IMAGE_PATH);



    cv::inRange(img, range.lower, range.upper, rangeMask);
    // dilation to fill gaps in the hand mask
    cv::dilate(rangeMask, rangeMask, cv::Mat(), cv::Point(-1, -1), 5); 

    // closing operation to fill small holes inside the foreground
    cv::morphologyEx(rangeMask, rangeMask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 3); 

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


/**
 * Initializes and configures a BackgroundSubtractorMOG2 instance.
 * @return A configured BackgroundSubtractorMOG2 instance.
 */
cv::Ptr<cv::BackgroundSubtractorMOG2> initBackgroundSubtractor() {
    // Create a BackgroundSubtractorMOG2 instance
    auto subtractor = cv::createBackgroundSubtractorMOG2();

    // Configure the background subtractor
    subtractor->setHistory(100);            // Number of frames for background learning
    subtractor->setVarThreshold(15);        // Threshold for foreground detection
    //subtractor->setDetectShadows(true);    // Disable shadow detection

    return subtractor;
}
// Gets the keypoints of the fingertips of the hand using Contour Detection and Convex Hull
class FastTracker : public HandTracker {
    public:
        // Background subtractor for removing the background, this object is stateful and should be reinitialized for each new video stream
        cv::Ptr<cv::BackgroundSubtractor> backgroundSubtractor;
        FastTracker() {
            backgroundSubtractor = initBackgroundSubtractor();
        }

        std::vector<cv::Point> getKeypoints(const cv::Mat& image){
            //cv::Mat foreground = applyMask(image, subtractBackground(image, backgroundSubtractor));

            cv::Mat handMask = makeHandMask(image);
            
            std::vector<cv::Point> contours = getHandContour(handMask);
            std::vector<int> fingertipIndices = getRoughHull(contours, MAX_DIST);

            std::vector<cv::Point> fingertipPoints = {};
            for (int idx : fingertipIndices) {
                fingertipPoints.push_back(contours[idx]);
            }
            cv::imshow("Hand Mask", handMask);


            // TODO: standardize the shape of the output to handle things that may retuirn more or less than all the hand keypoints
            return fingertipPoints;
        }
};

// prototype for deriving keypoints from the webcame stream frames
int main(){
    runHandTracking((new FastTracker()));
    return 0;
}