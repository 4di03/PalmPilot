#include "openposeTracker.h"
#include "cam.h"
#include <opencv2/core/ocl.hpp>
#include <chrono>
#define MODEL_PATH "data/models/pose_iter_102000.caffemodel"
#define PROTO_PATH "data/models/pose.prototxt"
#define W_IN 368
#define H_IN 368
#define NPARTS 22
#define THRESH 0.1
#define SCALE 0.003922
#define N_PAIRS 20
#define INTERP_INTERVAL 10 // extracts the keypoints every 10 frames


// represents the pairs of points that are connected in the hand for drwaing
const int HAND_POSE_PAIRS[N_PAIRS][2] = {   // hand
    {0,1}, {1,2}, {2,3}, {3,4},         // thumb
    {0,5}, {5,6}, {6,7}, {7,8},         // pinkie
    {0,9}, {9,10}, {10,11}, {11,12},    // middle
    {0,13}, {13,14}, {14,15}, {15,16},  // ring
    {0,17}, {17,18}, {18,19}, {19,20}   // small
};

/**
 * Draws the keypoints on the image
 * @param image the image to draw the keypoints on
 * @param keypoints the keypoints to draw (22,2) vector
 */
void drawKeypoints(cv::Mat& img, std::vector<cv::Point>& keypoints){

    for (int n = 0; n < N_PAIRS ; n++){
        // lookup 2 connected body/hand parts
        cv::Point2f a = keypoints[HAND_POSE_PAIRS[n][0]];
        cv::Point2f b = keypoints[HAND_POSE_PAIRS[n][1]];
        // we did not find enough confidence before
        if (a.x<=0 || a.y<=0 || b.x<=0 || b.y<=0)
            continue;

        line(img, a, b, cv::Scalar(0,200,0), 2);
        circle(img, a, 3, cv::Scalar(0,0,200), -1);
        circle(img, b, 3, cv::Scalar(0,0,200), -1);
    }
}

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

            // set the target to OpenCL to use the GPU
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);



        }
        std::vector<cv::Point> getKeypoints(const cv::Mat& image){

            // preprocess the image

            cv::Mat inputBlob = cv::dnn::blobFromImage(image, SCALE, cv::Size(W_IN, H_IN), cv::Scalar(0, 0, 0), false, false);


            net.setInput(inputBlob);

            // auto start = std::chrono::high_resolution_clock::now();
            cv::Mat result = net.forward();
            // auto end = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double, std::milli> elapsed = end - start;
            // std::cout << "network forward pass Execution time: " << elapsed.count() << " ms\n";
            // The forward pass of the network (this one line) takes > 99% of the execution time



            int H = result.size[2];
            int W = result.size[3];

            float SX = float(image.cols) / W;
            float SY = float(image.rows) / H;
            // the result is an array of "heatmaps", the probability of a body part being in location x,y
            std::vector<cv::Point> points(NPARTS);
            
            for( int n=0 ; n < NPARTS; n++){
                 // Slice heatmap of corresponding body's part.
                cv::Mat heatMap(H, W, CV_32F, result.ptr(0,n));
                // 1 maximum per heatmap
                cv::Point p(-1,-1),pm; // by default point is -1,-1 if it is not found
                double conf;
                cv::minMaxLoc(heatMap, 0, &conf, 0, &pm);
                if (conf > THRESH){
                    cv::Point curP(pm.x * SX,pm.y *  SY); // scale the point to the image size
                    p = curP;
                }

                points[n] = p;
            }
            
            return points;
        };
};

void printVector(std::vector<cv::Point>& vec){
    for (int i = 0; i < vec.size(); i++){
        std::cout << "Point " << i << ": ";
        cv::Point point = vec[i];
        std::cout << "X: " << point.x << " Y: " << point.y << std::endl;
    }
}


// prototype for deriving keypoints from the webcame stream frames
int main(){
    
    if (cv::ocl::haveOpenCL()) {
        std::cout << "OpenCL is available!" << std::endl;
        cv::ocl::setUseOpenCL(true);
    } else {
        std::cout << "OpenCL is not available on this system." << std::endl;
    }
    {
    VideoStream stream(0);
    int ct = 0;
    auto tracker = OpenPoseTracker();
    std::vector<cv::Point> keypoints = std::vector<cv::Point>(22,cv::Point(-1,-1));
    while (true) {
        ct++;
        cv::Mat frame = stream.getFrame();

        // Check if the frame is empty
        if (frame.empty()) {
            std::cerr << "Error: Failed to capture frame." << std::endl;
            break;
        }

        if (ct % INTERP_INTERVAL == 0){

            auto start = std::chrono::high_resolution_clock::now();
            keypoints = tracker.getKeypoints(frame);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed = end - start;
            std::cout << "getKeypoints Execution time: " << elapsed.count() << " ms\n";


            ct = 0; // reset the counter to avoid overflow
        }

        // if the keypoints are found, draw them on the image, else use the previous keypoints
        // auto start = std::chrono::high_resolution_clock::now();
        drawKeypoints(frame,keypoints);
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double, std::milli> elapsed = end - start;
        // std::cout << "drawKeypoints Execution time: " << elapsed.count() << " ms\n";


        cv::imshow("Webcam Stream with hands", frame);
        // Exit the loop when 'q' is pressed
        if (cv::waitKey(1) == 'q') {
            break;
        }

    }

    // Release the camera and destroy all windows
    cv::destroyAllWindows();

    }
    return 0;
}