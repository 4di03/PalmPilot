#include <opencv2/opencv.hpp>
#include "handTracker.h"
#pragma once
void drawKeypoints(cv::Mat& img, std::vector<cv::Point>& keypoints);
// runs hand tracker on vide stream and draws keypoints 
void plotHandKeypoints(HandKeypointTracker* tracker);

// Displays the hand data on the image
void displayHandData(cv::Mat& img, HandData& handData);

// gets hand data in each frame. videoPath: path to video file, empty string for webcam
void runHandTracking(HandTracker* tracker, std::string videoPath = "");