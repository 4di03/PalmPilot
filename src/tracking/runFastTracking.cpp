#include "fastTracker.h"
// prototype for deriving keypoints from the webcame stream frames
int main(){


    HandTracker* tracker = initBestTracker();
    runHandTracking(tracker);
    return 0;
}