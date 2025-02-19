#include <opencv2/opencv.hpp>
#define COLOR_RANGE_FILE "/Users/adithyapalle/work/PalmPilot/data/color_range.yaml"
#define TRACKING_BOX_FILE "/Users/adithyapalle/work/PalmPilot/data/tracking_box.yaml"
#pragma once
// Struct to represent color range
struct colorRange {
    cv::Scalar lower;
    cv::Scalar upper;

    std::string toString() const {
        return "Lower: " + std::to_string(lower[0]) + ", " + std::to_string(lower[1]) + ", " + std::to_string(lower[2]) + "\n" +
               "Upper: " + std::to_string(upper[0]) + ", " + std::to_string(upper[1]) + ", " + std::to_string(upper[2]);
    }
};






colorRange parseColorRange(std::string path);
cv::Rect parseTrackingBox(std::string path);

// updates teh color range in the file
void calibrateColorRange();

// updates the tracking box in the file
void calibrateTrackingBox();

void setBackground();