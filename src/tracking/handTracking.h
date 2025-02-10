#include <opencv2/opencv.hpp>
#include "handTracker.h"
#define FRAME_WIDTH 1280    
#define FRAME_HEIGHT 720
#pragma once
void drawKeypoints(cv::Mat& img, std::vector<cv::Point>& keypoints);
// runs hand tracker on vide stream and draws keypoints 
void plotHandKeypoints(HandKeypointTracker* tracker);
// gets hand data in each frame
void runHandTracking(HandTracker* tracker);