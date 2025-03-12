#include "kCurvature.h"
/**
 * Gets K-curvature data for each vertex of the convex hull of the contour.
 */
std::vector<KCurvatureData> getKCurvatureData(const std::vector<cv::Point>& contour, const std::vector<int>& hullIndices){
    if (contour.empty() || hullIndices.empty()){
        return std::vector<KCurvatureData>();
    }
    if (K_CURVATURE_POINTS >= 2* contour.size()){
        std::cerr << "Error: K_CURVATURE_POINTS must be less than half the size of the contour." << std::endl;
        exit(1);
    }
    std::vector<KCurvatureData> kCurvatureData;
    int n = contour.size();
    for (auto i : hullIndices){

        cv::Point start = contour[(i - K_CURVATURE_POINTS + n) % n];
        cv::Point point = contour[i];
        cv::Point end = contour[(i + K_CURVATURE_POINTS) % n];
        kCurvatureData.push_back(KCurvatureData{start, point, end,i});
    }



    return kCurvatureData;
}