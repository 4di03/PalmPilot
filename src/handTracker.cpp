#include "handTracker.h"
#include "cam.h"
// Uses opencv OpenPose to track the hand in the image
class OpenPoseTracker : public HandTracker{

    public:
        std::vector<std::vector<double>> getKeypoints(cv::Mat image){
            // Placeholder for the keypoints
            std::vector<std::vector<double>> keypoints;

            // TODO: use OpenPose to get the keypoints of the hand in the image

            
            return keypoints;
        };
};

void printVector(std::vector<std::vector<double>> vec){
    for (auto v : vec){
        for (auto i : v){
            std::cout << i << " ";
        }
        std::cout << std::endl;
    }
}

// prototype for deriving keypoints from the webcame stream frames
int main(){
    VideoStream stream(0);

    while (true) {
        cv::Mat frame = stream.getFrame();

        // Check if the frame is empty
        if (frame.empty()) {
            std::cerr << "Error: Failed to capture frame." << std::endl;
            break;
        }
        auto tracker = OpenPoseTracker();

        auto keypoints = tracker.getKeypoints(frame);

        std::cout << "Keypoints: " << std::endl;
        printVector(keypoints);
        

    }

    // Release the camera and destroy all windows
    cv::destroyAllWindows();



    return 0;
}