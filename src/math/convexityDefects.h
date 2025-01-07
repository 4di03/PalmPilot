#include <opencv2/opencv.hpp>


struct ConvexityDefect{
    
    cv::Point start;  // hull point at the start of the defect 
    cv::Point end; // hull point at the end of the defect
    cv::Point furthestPoint; // point in the middle of the defect that is the furthest from the convex hull
    double depth; // distance between the farthest point and the convex hull

    // get area of triangle formed by the start, end, and furthest point of the defect
    double getArea() const{
        
        return cv::norm(start - end) * depth / 2;
    }
};


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