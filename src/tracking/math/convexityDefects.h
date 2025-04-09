#pragma once
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

    /**
     * Determine if the triangle formed by the start, end, and furthest point of the defect is obtuse
     */
    bool isObtuse() const {
        cv::Point a = start - furthestPoint;
        cv::Point b = end - furthestPoint;
        cv::Point c = start - end;
    
        // Compute squared magnitudes to avoid redundant calculations
        double normA = cv::norm(a);
        double normB = cv::norm(b);
        double normC = cv::norm(c);
    
        // Compute angles using dot products
        double angle1 = acos(a.dot(b) / (normA * normB)); // Angle at furthestPoint
        double angle2 = acos((a).dot(c) / (normA * normC)); // Angle at start
        double angle3 = acos((-b).dot(c) / (normB * normC)); // Angle at end
    
        return angle1 > CV_PI / 2 || angle2 > CV_PI / 2 || angle3 > CV_PI / 2;
    }
    
};


/**
 * Gets the convexity defects of a contour
 * @param contour the contour to get the convexity defects of
 * @param hullIndices the indices of the convex hull points for the contour
 */
std::vector<ConvexityDefect> getConvexityDefects(const std::vector<cv::Point>& contour, const std::vector<int>& hullIndices, const std::vector<int>& fingertipIndices);
