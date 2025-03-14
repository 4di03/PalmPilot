### PalmPilot
Repository for coding computer control using hand pose tracking 

If you are interested in work related to building Deep Learning models to track hand pose information , see https://github.com/4di03/PalmPilotModels

### Usage

The palm detection relies on distinguishing the color of your palm, so please use this application in a background that contrasts your palm. Efforts were done using method that did not rely on color detection, but those were too slow to be feasible for real time tracking on a CPU-only device

### Local Setup

Dependencies:
- C++ 14
- OpenCV 4

Quick build and run command:

```
cd build && make && ./mouseControl
```

* this project is still a work in progress


### Parameters

A good color range I've found is:
```yaml
Lower: 0.000000, 146.000000, 0.000000
Upper: 255.000000, 254.000000, 255.000000
```