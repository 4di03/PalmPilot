#include "convexityDefects.h"
#include "constants.h"

// Comparator for cv::Point to be used in std::set
struct PointComparator {
    /**
     * Compares two cv::Points, returning true if p1 < p2, false otherwise
     */
    bool operator()(const cv::Point& p1, const cv::Point& p2) const {
        return std::tie(p1.x, p1.y) < std::tie(p2.x, p2.y);
    }
};

/**
 * TODO: get the convexity defects from the entire hull, but filter those that are not direclty attached to any finger tips (as those tend to 
 * intersect with the palm area), and then you can use these accurate defects to determine the slenderness/angle of the triangels
 * of them to see if the hand is raised or not.
 * 
 * Gets the convexity defects of a contour
 * @param contour the contour to get the convexity defects of
 * @param hullIndices the indices of the convex hull points for the contour
 * @param fingertipIndices the indices of the fingertip points for the contour (subset of hullIndices)
 */
std::vector<ConvexityDefect> getConvexityDefects(const std::vector<cv::Point>& contour, const std::vector<int>& hullIndices, const std::vector<int>& fingertipIndices){
    if (contour.empty() || hullIndices.empty()){
        std::cerr << "Error: Empty contour passed to getConvexityDefects" << std::endl;
        exit(1);
    }


    // get a set of the fingertip points
    std::set<cv::Point, PointComparator> pointSet;
    for (int i : fingertipIndices){
        pointSet.insert(contour[i]);
    }

    std::vector<cv::Vec4i> rawDefects;
    std::vector<ConvexityDefect> defects;



    cv::convexityDefects(contour, hullIndices, rawDefects);

    if (DEBUG){
        printf("Number of hull indices: %lu\n", hullIndices.size());
        printf("Number of raw defects: %lu\n", rawDefects.size());
    }

    for (cv::Vec4i defect : rawDefects){
        ConvexityDefect cd;

        cv::Point start = contour[defect[0]];
        cv::Point end = contour[defect[1]];

        // skip if neither of the hull points for the defect are fingertip points
        if (pointSet.find(start) == pointSet.end() && pointSet.find(end) == pointSet.end()){
            continue;
        }


        cd.start = contour[defect[0]];
        cd.end = contour[defect[1]];
        cd.furthestPoint = contour[defect[2]];
        cd.depth = defect[3] / 256.0;
        defects.push_back(cd);
    }



    return defects;
}