#include <opencv2/opencv.hpp>
#include "handTracker.h"
#pragma once
void drawKeypoints(cv::Mat& img, std::vector<cv::Point>& keypoints);
// runs hand tracker on vide stream and draws keypoints 
void plotHandKeypoints(HandKeypointTracker* tracker);

// Displays the hand data on the image
void displayHandData(cv::Mat& img, HandData& handData);

// gets hand data in each frame
void runHandTracking(HandTracker* tracker);