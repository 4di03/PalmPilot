#include "kCurvature.h"
/**
 * Gets K-curvature data for each vertex of the convex hull of the contour.
 */
std::vector<KCurvatureData> getKCurvatureData(const std::vector<cv::Point>& contour, const std::vector<int>& hullIndices){
    std::vector<KCurvatureData> kCurvatureData;
    int n = contour.size();
    for (auto i : hullIndices){
        cv::Point start = contour[(i - K + n) % n];
        cv::Point point = contour[i];
        cv::Point end = contour[(i + K) % n];
        kCurvatureData.push_back(KCurvatureData{start, point, end});
    }
    return kCurvatureData;
}