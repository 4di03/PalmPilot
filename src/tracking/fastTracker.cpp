#include "fastTracker.h"
#include "cam.h"
#include "calibration.h"
/**
 * Computes the circularity of a contour by comparing the area and perimeter of its convex hull
 */
float getConvexHullCircularity(const std::vector<cv::Point> &contour)
{

    // Compute Convex Hull
    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);

    // Compute Area and Perimeter of the Convex Hull
    double hullArea = cv::contourArea(hull);
    double hullPerimeter = cv::arcLength(hull, true);

    // Compute Circularity: 4π * Area / Perimeter²
    if (hullPerimeter == 0)
        return false; // Prevent division by zero
    return (4 * CV_PI * hullArea) / (hullPerimeter * hullPerimeter);
}

void printMat(const cv::Mat &mat)
{
    for (int i = 0; i < mat.rows; i++)
    {
        for (int j = 0; j < mat.cols; j++)
        {
            // For singlfe-channel matrices
            if (mat.channels() == 1)
            {
                std::cout << mat.at<uchar>(i, j) << " ";
            }
            // For multi-channel matrices
            else if (mat.channels() == 3)
            {
                cv::Vec3b pixel = mat.at<cv::Vec3b>(i, j);
                std::cout << "[" << (int)pixel[0] << ", " << (int)pixel[1] << ", " << (int)pixel[2] << "] ";
            }
        }
        std::cout << std::endl;
    }
}
// gets a lower and upper range of the  colors given the average color of the palm
colorRange getRangeFromImage(std::string imagePath)
{
    cv::Mat image = cv::imread(imagePath);
    cv::Mat img;
    cv::cvtColor(image, img, COLOR_CONVERSION);
    cv::Scalar mean, stddev;
    cv::meanStdDev(img, mean, stddev);

    // printMat(img);

    // std:: cout << "Mean: " << mean << std::endl;
    // std:: cout << "Stddev: " << stddev << std::endl;
    cv::Scalar lower(mean[0] - ST_DEVS_DIFF * stddev[0], mean[1] - ST_DEVS_DIFF * stddev[1], mean[2] - ST_DEVS_DIFF * stddev[2]);
    cv::Scalar upper(mean[0] + ST_DEVS_DIFF * stddev[0], mean[1] + ST_DEVS_DIFF * stddev[1], mean[2] + ST_DEVS_DIFF * stddev[2]);

    // std::cout << "Lower: " << lower << std::endl;
    // std::cout << "Upper: " << upper << std::endl;
    return colorRange{lower, upper};
}

// gets contour with largest area
std::vector<cv::Point> MaxAreaFilter::filterContour(std::vector<std::vector<cv::Point>> contours)
{
    if (contours.empty())
    {   
        if (DEBUG){
            std::cout << "No contours passed to MaxAreaFilter" << std::endl;
        }
        return std::vector<cv::Point>();
    }
    std::vector<cv::Point> maxContour = *(std::max_element(contours.begin(), contours.end(),
                                                           [](const std::vector<cv::Point> &c0, const std::vector<cv::Point> &c1)
                                                           {
                                                               return cv::contourArea(c0) < cv::contourArea(c1);
                                                           }));

    return maxContour;
}
// gets contouor with ratio of contour area to convex hull area that is between a certain range, if many exist in that range it, then it applies a secondary filter to get the best one
std::vector<std::vector<cv::Point>> SolidityFilter::getValidContours(std::vector<std::vector<cv::Point>> contours)
{
    if (contours.empty())
    {
        return std::vector<std::vector<cv::Point>>();
    }
    // Sort contours by area in descending order and return the largest one

    std::vector<std::vector<cv::Point>> validContours;
    for (auto contour : contours)
    {
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double area = cv::contourArea(contour);
        double hullArea = cv::contourArea(hull);
        double solidity = area / hullArea;
        if (solidity > MIN_SOLIDITY && solidity < MAX_SOLIDITY)
        {
            validContours.push_back(contour);
        }
    }
    return validContours;
}
// filters out contours that take up too much or too little of the image
std::vector<std::vector<cv::Point>> AreaFilter::getValidContours(std::vector<std::vector<cv::Point>> contours)
{
    if (contours.empty())
    {
        if(DEBUG){
            std::cout << "No contours passed to AreaFilter" << std::endl;
        }
        return std::vector<std::vector<cv::Point>>();
    }
    // Sort contours by area in descending order and return the largest one

    std::vector<std::vector<cv::Point>> validContours;

    static TrackingRect trackingBox = parseTrackingBox(TRACKING_BOX_FILE);
    double roiArea = trackingBox.width() * trackingBox.height();
    for (auto contour : contours)
    {
        double prop = cv::contourArea(contour) / roiArea;

        if (prop > MIN_PROP)
        {

            validContours.push_back(contour);
        }
    }
    return validContours;
}

/**
 * Filters out contours for which he convex hull is too circular
 */
std::vector<std::vector<cv::Point>> CircularityFilter::getValidContours(std::vector<std::vector<cv::Point>> contours)
{
    if (contours.empty())
    {   
        if(DEBUG){
            std::cout << "No contours passed to CircularityFilter" << std::endl;
        }
        return std::vector<std::vector<cv::Point>>();
    }
    // Sort contours by area in descending order and return the largest one

    std::vector<std::vector<cv::Point>> validContours;
    for (auto contour : contours)
    {
        float circularity = getConvexHullCircularity(contour);
        if(DEBUG){
            std::cout << "Circularity: " << circularity << std::endl;
        }
        if (circularity < CIRCULARITY_THRESHOLD)
        {
            validContours.push_back(contour);
        }
    }

    return validContours;
}


std::vector<cv::Point> CompositeFilter::filterContour(std::vector<std::vector<cv::Point>> contours)
{
    std::vector<std::vector<cv::Point>> validContours(contours);

    // Apply each filter in sequence until a valid contour is found
    for (auto filter : validFilters)
    {
        validContours = filter->getValidContours(validContours);
        if (validContours.empty())
        { // no need to apply more filters if no valid contours are found
            break;
        }
    }

    // apply the final filter
    return finalSelector->filterContour(validContours);
}

GaussianBlurPostProcessing::GaussianBlurPostProcessing(int blurSize)
{
    this->blurSize = blurSize;
}
cv::Mat GaussianBlurPostProcessing::postProcess(cv::Mat &mask)
{
    cv::GaussianBlur(mask, mask, cv::Size(blurSize, blurSize), 0);
    return mask;
}

cv::Mat DilationPostProcessing::postProcess(cv::Mat &mask)
{
    cv::dilate(mask, mask, this->kernel, cv::Point(-1, -1), dilationIterations);
    return mask;
}


cv::Mat ClosingPostProcessing::postProcess(cv::Mat &mask)
{
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, this->kernel, cv::Point(-1, -1), closingIterations);
    return mask;
}
// applies a series of post processing strategies to the mask
CompositePostProcessing::CompositePostProcessing(std::vector<HandMaskPostProcessingStrategy *> postProcessingStrategies)
{
    this->postProcessingStrategies = postProcessingStrategies;
}

cv::Mat CompositePostProcessing::postProcess(cv::Mat &mask)
{
    for (auto strategy : postProcessingStrategies)
    {
        mask = strategy->postProcess(mask);
    }
    return mask;
}

ErosionPostProcessing::ErosionPostProcessing(int erosionIterations)
{
    this->erosionIterations = erosionIterations;
}

cv::Mat ErosionPostProcessing::postProcess(cv::Mat &mask)
{
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), erosionIterations);
    return mask;
}

cv::Mat IdentityPostProcessing::postProcess(cv::Mat &mask)
{
    return mask;
}

HandMaskStrategy::HandMaskStrategy(HandMaskPostProcessingStrategy *postProcessingStrategy)
{
    this->postProcessingStrategy = postProcessingStrategy;
}
cv::Mat HandMaskStrategy::makeHandMask(const cv::Mat &image)
{
    static colorRange range = parseColorRange(COLOR_RANGE_FILE);

    // Filter by skin color
    cv::Mat img;
    cv::cvtColor(image, img, COLOR_CONVERSION);

    cv::Mat rangeMask;

    cv::inRange(img, range.lower, range.upper, rangeMask);
    if (DEBUG)
    {
        // cv::imshow("Raw Mask", rangeMask);
    }

    return postProcessingStrategy->postProcess(rangeMask);
};

double dist(cv::Point a, cv::Point b)
{
    return cv::norm(a - b);
}

/**
 * Applies a binary mask to an image.
 * @param image The input image (original frame).
 * @param mask The binary mask (single channel).
 * @return The masked image with the background removed.
 */
cv::Mat applyMask(const cv::Mat &image, const cv::Mat &mask)
{
    cv::Mat result;

    // Ensure the mask has a single channel
    if (mask.channels() != 1)
    {
        throw std::invalid_argument("Mask must be a single-channel binary image");
    }

    // Ensure the mask and image have the same size
    if (image.size() != mask.size())
    {
        throw std::invalid_argument("Image and mask sizes must match");
    }

    // Apply the mask to the image
    cv::bitwise_and(image, image, result, mask);

    return result;
}
// Segmenting by skin color
auto skinColorUpper = [](int hue)
{ return cv::Scalar(hue, 0.5 * 255, 0.6 * 255); };
auto skinColorLower = [](int hue)
{ return cv::Scalar(hue, 0.1 * 255, 0.05 * 255); };

// Function to calculate the distance between two points
double ptDist(const cv::Point &pt1, const cv::Point &pt2)
{
    return cv::norm(pt1 - pt2);
}

// Function to get the center point of a group of points
cv::Point getCenterPt(const std::vector<cv::Point> &points)
{
    cv::Point center(0, 0);
    for (const auto &pt : points)
    {
        center += pt;
    }
    center.x /= points.size();
    center.y /= points.size();
    return center;
}

// Function to get the indices of points representing finger tips (first find the rough convex hull and then search for nearby points that have maximum y-coordinate)
// Returns the contour indices for the most central points, sorted in ascending order
std::vector<int> getFingertipPoints(const std::vector<cv::Point> &contour, double maxDist)
{

    // Get hull indices and hull points
    std::vector<int> hullIndices;
    if (contour.empty())
    {
        std::cerr << "Error: Input to convexHull is empty." << std::endl;
        return hullIndices;
    }

    cv::convexHull(contour, hullIndices, false, false);
    std::vector<cv::Point> hullPoints;
    for (int idx : hullIndices)
    {
        hullPoints.push_back(contour[idx]);
    }

    // Group all points in local neighborhood
    auto ptsBelongToSameCluster = [&](const cv::Point &pt1, const cv::Point &pt2)
    {
        return ptDist(pt1, pt2) < maxDist;
    };
    std::vector<int> labels;
    cv::partition(hullPoints, labels, ptsBelongToSameCluster);

    std::map<int, std::vector<std::pair<cv::Point, int>>> pointsByLabel;
    for (size_t i = 0; i < labels.size(); ++i)
    {
        pointsByLabel[labels[i]].emplace_back(hullPoints[i], hullIndices[i]);
    }

    // Map points in local neighborhood to most central point
    auto getMostCentralPoint = [&](const std::vector<std::pair<cv::Point, int>> &pointGroup)
    {
        // Find center
        std::vector<cv::Point> points;
        for (const auto &ptWithIdx : pointGroup)
        {
            points.push_back(ptWithIdx.first);
        }
        cv::Point center = getCenterPt(points);

        // Sort ascending by distance to center
        auto compare = [&](const std::pair<cv::Point, int> &ptWithIdx1, const std::pair<cv::Point, int> &ptWithIdx2)
        {
            return ptDist(ptWithIdx1.first, center) < ptDist(ptWithIdx2.first, center);
        };
        return *std::min_element(pointGroup.begin(), pointGroup.end(), compare);
    };

    std::vector<int> result;
    for (const auto &pointGroup : pointsByLabel)
    {
        result.push_back(getMostCentralPoint(pointGroup.second).second);
    }

    // find nearby points (within K) to hull points that have minimum y-coordinate (highest up) (these are the fingertips)

    std::set<int> fingertips; // Ordered set for efficient nearest neighbor lookup

    for (int i = 0; i < result.size(); i++)
    {
        int curIdx = result[i];
        int minIdx = curIdx;

        // Find nearby point within [curIdx - K, curIdx + K] that has maximum y-coordinate
        int lower = std::max(0, curIdx - K);
        int upper = std::min((int)contour.size(), curIdx + K + 1);
        for (int j = lower; j < upper; j++)
        {
            if (contour[j].y < contour[minIdx].y)
            { // Find highest point (lowest y)
                minIdx = j;
            }
        }

        // Check if any existing fingertip is within range K
        auto it = fingertips.lower_bound(minIdx - K);
        if (it != fingertips.end() && *it <= minIdx + K)
        {
            continue; // Skip adding if a nearby point already exists
        }

        fingertips.insert(minIdx); // Add the new fingertip
    }

    return std::vector<int>(fingertips.begin(), fingertips.end());
}

/**
 * Checks if a line from points a to b intersects a circle with center c and radius r.align
 *
 * Works by finding the closest point on AB to the center of the circle,
 * and then checking if the distance from that point to the center is less than the radius of the circle
 * TODO: understand this math better.
 * @param A The start point of the line
 * @param B The end point of the line
 * @param circle The circle to check for intersection
 * @return True if the line intersects the circle, false otherwise
 */
bool doesLineIntersectCircle(const cv::Point2f &A, const cv::Point2f &B, const Circle circle)
{
    float radius = circle.radius;
    cv::Point center = circle.center;

    // Convert to floating-point for precision
    cv::Point2f C = cv::Point2f(center);

    // Vector from A to B
    cv::Point2f AB = B - A;

    // Vector from A to Circle Center
    cv::Point2f AC = C - A;

    // Project AC onto AB to find how far along AB the closest point to C is
    float t = (AC.dot(AB)) / (AB.dot(AB));

    // Clamp t to stay within the segment
    t = std::max(0.0f, std::min(1.0f, t));

    // Closest point on segment to the circle
    cv::Point2f closestPoint = A + t * AB;

    // Check distance from closestPoint to circle center (d^2)
    float distanceSquared = (closestPoint.x - C.x) * (closestPoint.x - C.x) +
                            (closestPoint.y - C.y) * (closestPoint.y - C.y);

    return distanceSquared <= radius * radius; // True if the segment intersects the circle.
    // if you square root both sides, you are basically chceking that d < r
}

/**
 *
 *
 * Gets the position of the index finger form the convexity defects on the convex hull of the hand by . . .
 *  1. filter out convexity defects with depth smaller than the radius of the max inscribing circle (If no valid fingers are found, return (-1, -1))
 *  2. get the convexity defect with the largest area
 *  3. designate the taller of the two points of the defect as the index finger position (because thumb is smaller). Height is the distance from the furthest point to the start or end point of the defect
 *
 */
cv::Point getIndexFingerPosition(std::vector<ConvexityDefect> convexityDefects, Circle maxInscribingCircle)
{
    // filter out convexity defects with depth smaller than the radius of the max inscribing circle, or if the defects outer line cross the max inscribing circle
    convexityDefects.erase(std::remove_if(convexityDefects.begin(), convexityDefects.end(),
                                          [&](const ConvexityDefect &cd)
                                          {
                                              return cd.depth < maxInscribingCircle.radius || doesLineIntersectCircle(cd.start, cd.end, maxInscribingCircle);
                                          }),
                           convexityDefects.end());

    if (convexityDefects.empty())
    {
        // means no valid fingers were found
        return cv::Point(-1, -1);
    }

    // get the convexity defect with the largest area

    ConvexityDefect maxDefect = *std::max_element(convexityDefects.begin(), convexityDefects.end(),
                                                  [](const ConvexityDefect &cd1, const ConvexityDefect &cd2)
                                                  {
                                                      return cd1.getArea() < cd2.getArea();
                                                  });

    cv::Point indexFingerPosition;
    if (dist(maxDefect.start, maxDefect.furthestPoint) > dist(maxDefect.end, maxDefect.furthestPoint))
    { // index finger is the one that is furthest  from defect
        indexFingerPosition = maxDefect.start;
    }
    else
    {
        indexFingerPosition = maxDefect.end;
    }
    return indexFingerPosition;
}

/**
 * Gets Hand Data from the contour
 * 
 */
HandData getHandDataFromContour(const std::vector<cv::Point> &contour, const cv::Mat &img = cv::Mat())
{
    if (contour.size() == 0)
    {
        if (DEBUG){
            std::cerr << "Error: Empty contour passed to getHandDataFromContour" << std::endl;
        }
        return HandData{cv::Point(-1, -1), 0, false};
    }

    // max inscribing circle is assumed to be the palm of the hand
    Circle maxInscribingCircle = getMaxInscribingCircle(contour, img);

    if (DEBUG)
    {
        // cv::Mat inscribingCircleImage = img.clone();
        // cv::circle(inscribingCircleImage, maxInscribingCircle.center, maxInscribingCircle.radius, cv::Scalar(0, 255, 0), 2);
        // cv::imshow("Max Inscribing Circle", inscribingCircleImage);
    }

    // filter contours in region 4 * radius of max inscribing circle (MIC), and are above the lowest point of the MIC
    std::vector<cv::Point> newContour;
    for (cv::Point pt : contour)
    {
        if (dist(pt, maxInscribingCircle.center) < MAX_INSCRIBING_CIRCLE_CONTOUR_DIST * maxInscribingCircle.radius &&
            pt.y < maxInscribingCircle.center.y + maxInscribingCircle.radius)
        {
            newContour.push_back(pt);
        }
    }

    // get convex hull of the new contour
    std::vector<int> fingertipIndices = getFingertipPoints(newContour, MAX_DIST);
    if (fingertipIndices.size() == 0)
    {
        // even though no finger was found , we assume the contour still represents the hand as it passed through the contour filters
        return HandData{cv::Point(-1, -1), 0, true};
    }
    std::vector<KCurvatureData> kCurvatures = getKCurvatureData(newContour, fingertipIndices);

    if (DEBUG)
    {

        // draw new contour
        cv::Mat newContourImage = img.clone();
        cv::drawContours(newContourImage, std::vector<std::vector<cv::Point>>{newContour}, -1, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Hand Contour", newContourImage);

        // draw k-curvatures
        cv::Mat KCurvatureImage = img.clone();
        for (KCurvatureData kCurvature : kCurvatures)
        {
            cv::line(KCurvatureImage, kCurvature.start, kCurvature.point, cv::Scalar(0, 255, 0), 2);
            cv::line(KCurvatureImage, kCurvature.end, kCurvature.point, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("K Curvature", KCurvatureImage);
    }

    std::vector<cv::Point> fingertipPoints;

    for (KCurvatureData kCurvature : kCurvatures)
    {
        double angle = kCurvature.getKCurvature();
        if (angle > MIN_CURVATURE && angle < MAX_CURVATURE)
        {
            fingertipPoints.push_back(kCurvature.point);
        }
    }

    if (DEBUG)
    {
        cv::Mat fingertipImage = img.clone();
        drawKeypoints(fingertipImage, fingertipPoints);
        cv::imshow("Fingertips", fingertipImage);
    }

    std::vector<ConvexityDefect> convexityDefects = getConvexityDefects(newContour, fingertipIndices);

    if (DEBUG)
    {
        // shade the convexity defects with a unique color for each
        cv::Mat convexityDefectImage = img.clone();
        for (int i = 0; i < convexityDefects.size(); i++)
        {
            ConvexityDefect cd = convexityDefects[i];
            // make random color
            cv::Scalar color = cv::Scalar(rand() % 256, rand() % 256, rand() % 256);
            cv::line(convexityDefectImage, cd.start, cd.end, color, 2);
            cv::line(convexityDefectImage, cd.start, cd.furthestPoint, color, 2);
            cv::line(convexityDefectImage, cd.end, cd.furthestPoint, color, 2);
        }
        cv::imshow("Convexity Defects", convexityDefectImage);
    }

    cv::Point indexFingerPosition = getIndexFingerPosition(convexityDefects, maxInscribingCircle);

    return HandData{indexFingerPosition, static_cast<int>(fingertipPoints.size()), true};
}

FastTracker::FastTracker(ContourFilterStrategy *filterStrategy, HandMaskStrategy *maskStrategy)
{
    this->filterStrategy = filterStrategy;
    this->maskStrategy = maskStrategy;
}

/**
 * Gets the HandData relative to the tracking box from the image (defined in TRACKING_BOX_FILE)
 * @param image The image to get the hand data from
 * @return HandData object containing the hand data
 */
HandData FastTracker::getHandData(const cv::Mat &image)
{

    // slice only the region of interest (tracking box)
    static TrackingRect trackingBox = parseTrackingBox(TRACKING_BOX_FILE);
    cv::Mat roi = trackingBox.cropImage(image);
    cv::Mat handMask = maskStrategy->makeHandMask(roi);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(handMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (DEBUG){
        std::cout << "Contours size pre-filter: " << contours.size() << std::endl;
    }

    std::vector<cv::Point> contour = filterStrategy->filterContour(contours);


    if (DEBUG)
    {
        std::cout<<  "Contour size: " << contour.size() << std::endl;
        // // draw the contour
        // cv::Mat contourImage = cv::Mat::zeros(handMask.size(), CV_8UC3);
        // cv::drawContours(contourImage, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);

        // cv::imshow("Hand Mask", handMask);
    }

    HandData h =  getHandDataFromContour(contour, roi);

    // invalidate the data if the index finger is not found in the top box
    if (h.indexFingerPosition.x != -1 && h.indexFingerPosition.y > trackingBox.screenHeight())
    {
        return HandData{cv::Point(-1, -1), 0, false};
    }else{
        return h;
    }
}

/**
 * Factory function to create a new FastTracker instance
 */

FastTracker* initBestTracker() {
    HandMaskStrategy* maskStrategy = new HandMaskStrategy(
        new CompositePostProcessing(
            {
             new DilationPostProcessing(1), 
            }
        )
    );
    ContourFilterStrategy* filter = new CompositeFilter({new AreaFilter()}, new MaxAreaFilter());
    return (new FastTracker(filter, maskStrategy));
}