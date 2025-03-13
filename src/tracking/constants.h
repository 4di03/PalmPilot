#define DEBUG true

// Constants
#define MAX_DIST 30
#define INTERVAL 20
#define ST_DEVS_DIFF 5
#define BLUR_SIZE 10
#define MIN_SOLIDITY 0.5
#define MAX_SOLIDITY 0.7
#define MIN_PROP 0.01
#define MAX_PROP 0.3
#define COLOR_CONVERSION cv::COLOR_BGR2YCrCb
// min and max angles (degrees) for segments from hull points that could be considered fingertips

#define MIN_CURVATURE 10
#define MAX_CURVATURE 90

#define K_CURVATURE_POINTS 20// number of points used to make vectors to find k curvature from the vertices of the convex hull




#define CIRCULARITY_THRESHOLD 0.77 
#define MAX_INSCRIBING_CIRCLE_CONTOUR_DIST 6