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


/**
 * Represents a tracking rectangle where the top box (defined by top and middle points) represents the space in which mouse position translates to screen position
 * The entire space is in which the hand is detected, so the bottom box is necessary to make sure the finger can be detected at the bottom of the top box
 */
class TrackingRect{
    public:
        cv::Point topLeft;
        int middleY;
        cv::Point bottomRight;
        

        TrackingRect() : topLeft(0, 0), middleY(0), bottomRight(0, 0) {}

        /***
         * 
         *  Validates the input to make sure it looks like:
         *  
         *  topLeft - - - - - - - - - - - - 
         *    |                           |
         *    |                           |
         *    | - - - - - middleY - - - - |
         *    |                           |
         *    |                           |
         *    |                           |
         *    - - - - - - - - - - - - bottomRight
         * 
         */
        TrackingRect(cv::Point topLeft, int middleY, cv::Point bottomRight){
            if (bottomRight.x <= topLeft.x || bottomRight.y <= topLeft.y || middleY <= topLeft.y || middleY >= bottomRight.y){
                throw std::invalid_argument("Invalid tracking box dimensions");
            }
            this->topLeft = topLeft;
            this->middleY = middleY;
            this->bottomRight = bottomRight;
        }

        // width of the entire box
        float width() const {
            return bottomRight.x - topLeft.x;
        }
        
        // height of the entire box
        float height() const {
            return bottomRight.y - topLeft.y;
        }
        // width of top box
        float screenWidth() const {
            return this->width();
        }   

        // height of top box
        float screenHeight() const {
            return middleY - topLeft.y;
        }

        /**
         * Draws the tracking box on the image, modifying the image
         */
        cv::Mat draw(cv::Mat& image) const {
            //draw the top box
            cv::rectangle(image, topLeft, cv::Point(bottomRight.x, middleY), cv::Scalar(0, 255, 0), 2);
            // draw the bottom box
            cv::rectangle(image, cv::Point(topLeft.x, middleY), bottomRight, cv::Scalar(0, 255, 0), 2);

            return image;
        }


        /**
         * Crops the image to the tracking box
         */
        cv::Mat cropImage(const cv::Mat& image) const {
            return image(cv::Rect(topLeft, bottomRight)).clone();
        }


        bool contains(const cv::Point& point) const {
            return point.x >= topLeft.x && point.x <= bottomRight.x && point.y >= topLeft.y && point.y <= bottomRight.y;
        }

        
};



colorRange parseColorRange(std::string path);
TrackingRect parseTrackingBox(std::string path);

// updates teh color range in the file
void calibrateColorRange();

// updates the tracking box in the file
void calibrateTrackingBox();

void setBackground();