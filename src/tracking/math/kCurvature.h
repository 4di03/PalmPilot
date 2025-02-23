#include <opencv2/opencv.hpp>
#define K_CURVATURE_POINTS 20 // number of points used to make vectors to find k curvature from the vertices of the convex hull
/**
 * Data used to measure the local curvature of a contour at a given point by taking vector1 (start to point) and vector2 (point to end).
 * The K-curvature is the angle between vector1 and vector2.
 */
struct KCurvatureData{
    const cv::Point start;
    const cv::Point point;
    const cv::Point end;


    /**
     * Gets the angle theta (degrees) between vector1 (start to point) and vector2 (point to end).
     * 
     * dot(A,B) = |A| * |B| * cos(theta)
     * theta = acos(dot(A,B) / (|A| * |B|))
     */
    double getKCurvature() const{
        cv::Point vector1 = start - point;
        cv::Point vector2 = end - point;
        double dotProduct = vector1.x * vector2.x + vector1.y * vector2.y;
        double magnitude1 = sqrt(vector1.x * vector1.x + vector1.y * vector1.y);
        double magnitude2 = sqrt(vector2.x * vector2.x + vector2.y * vector2.y);
        return (acos(dotProduct / (magnitude1 * magnitude2))) * (180 / CV_PI);
    }
};


/**
 * Gets K-curvature data for each vertex of the convex hull of the contour.
 */
std::vector<KCurvatureData> getKCurvatureData(const std::vector<cv::Point>& contour, const std::vector<int>& hullIndices);