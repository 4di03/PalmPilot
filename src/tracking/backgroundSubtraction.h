#include <opencv2/opencv.hpp>
#include "handTracking.h"
#define THRESH_MIN 0
#define THRESH_MAX 255
#define FOREGROUND_DILATION_ITERATIONS 10
#define BACKGROUND_FILE_LOC "/Users/adithyapalle/work/PalmPilot/data/background.png"

cv::Mat initBackground();
// Perform background subtraction by subtracting the background from the frame, and retursn the foreground
cv::Mat backgroundSubtraction(const cv::Mat& background, const cv::Mat& frame);