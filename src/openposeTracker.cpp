#include "openposeTracker.h"
#include "cam.h"
#define MODEL_PATH "data/models/pose_iter_102000.caffemodel"
#define PROTO_PATH "data/models/pose.prototxt"
#define W_IN 368
#define H_IN 368
#define NPARTS 22
#define THRESH 0.1
#define SCALE 0.003922



// Uses opencv OpenPose to track the hand in the image
class OpenPoseTracker : public openposeTracker{
    private:
        cv::dnn::Net net;
    public:
        // initalizes network       
        OpenPoseTracker(){
            cv::String modelTxt = cv::samples::findFile(PROTO_PATH);
            cv::String modelBin = cv::samples::findFile(MODEL_PATH);
            if (modelTxt.empty() || modelBin.empty())
            {
                std::cerr << "Can't read model files. Please check the path to the model files." << std::endl;
                exit(-1);
            }

            net = cv::dnn::readNet(modelBin, modelTxt);




        }
        std::vector<cv::Point> getKeypoints(cv::Mat image){
            // preprocess the image
            cv::Mat inputBlob = cv::dnn::blobFromImage(image, SCALE, cv::Size(W_IN, H_IN), cv::Scalar(0, 0, 0), false, false);

            net.setInput(inputBlob);
            cv::Mat result = net.forward();
            int H = result.size[2];
            int W = result.size[3];
            // the result is an array of "heatmaps", the probability of a body part being in location x,y
            std::vector<cv::Point> points(NPARTS);
            
            for( int n=0 ; n < NPARTS; n++){
                 // Slice heatmap of corresponding body's part.
                cv::Mat heatMap(H, W, CV_32F, result.ptr(0,n));
                // 1 maximum per heatmap
                cv::Point p(-1,-1),pm; // by default point is -1,-1 if it is not found
                double conf;
                cv::minMaxLoc(heatMap, 0, &conf, 0, &pm);
                if (conf > THRESH)
                    p = pm;
                points[n] = p;
            }
            
            return points;
        };
};

void printVector(std::vector<cv::Point> vec){
    for (int i = 0; i < vec.size(); i++){
        std::cout << "Point " << i << ": ";
        cv::Point point = vec[i];
        std::cout << "X: " << point.x << " Y: " << point.y << std::endl;
    }
}

// prototype for deriving keypoints from the webcame stream frames
int main(){
    {
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

    }
    return 0;
}