#include "fastTracker.h"
#include <string>
// prototype for deriving keypoints from the webcam stream frames
// Usage: fast_tracking [--video <path>]
int main(int argc, char** argv){

    std::string videoPath = "";
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--video" && i + 1 < argc) {
            videoPath = argv[i + 1];
        }
    }

    HandTracker* tracker = initBestTracker();
    runHandTracking(tracker, videoPath);
    return 0;
}