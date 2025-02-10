#include <opencv2/opencv.hpp>

struct HandData{
    cv::Point indexFingerPosition; // point of the index finger if it and the thumb are raised, (-1,-1) if it is not raised
    int numFingersRaised; // number of fingers raised, between 0 and 5
    bool handDetected; // True if the hand contour is in frame, else False
};