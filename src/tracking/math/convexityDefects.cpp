#include "convexityDefects.h"

/**
 * Gets the convexity defects of a contour
 * @param contour the contour to get the convexity defects of
 * @param hullIndices the indices of the convex hull points for the contour
 */
std::vector<ConvexityDefect> getConvexityDefects(const std::vector<cv::Point>& contour, const std::vector<int>& hullIndices){
    std::vector<cv::Vec4i> rawDefects;
    std::vector<ConvexityDefect> defects;

    cv::convexityDefects(contour, hullIndices, rawDefects);

    for (cv::Vec4i defect : rawDefects){
        ConvexityDefect cd;
        cd.start = contour[defect[0]];
        cd.end = contour[defect[1]];
        cd.furthestPoint = contour[defect[2]];
        cd.depth = defect[3] / 256.0;
        defects.push_back(cd);
    }
    return defects;
}