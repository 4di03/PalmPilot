#include <opencv2/opencv.hpp>
#include "handTracker.h"
#pragma once
void drawKeypoints(cv::Mat& img, std::vector<cv::Point>& keypoints);
// runs hand tracker on vide stream and draws keypoints 
void runHandTracking(HandTracker* tracker);