#include "fastTracker.h"
// prototype for deriving keypoints from the webcame stream frames
int main(){

    HandMaskStrategy* maskStrategy = new HandMaskStrategy(
        new CompositePostProcessing(
            {
             new DilationPostProcessing(1), 
            }
        )
    );
    ContourFilterStrategy* filter = new CompositeFilter({new AreaFilter(), new CircularityFilter()}, new MaxAreaFilter());
    HandTracker* tracker = new FastTracker(filter, maskStrategy);

    runHandTracking(tracker);
    return 0;
}