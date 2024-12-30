#include "handTracker.h"
#include "handTracking.h"
#include <chrono>
#include <opencv2/opencv.hpp>

#define MODEL_PATH "data/models/pose_iter_102000.caffemodel"
#define PROTO_PATH "data/models/pose.prototxt"
#define W_IN 368
#define H_IN 368
#define NPARTS 22
#define THRESH 0.1
#define SCALE 0.003922
#define N_PAIRS 20


// represents the pairs of points that are connected in the hand for drwaing
const int HAND_POSE_PAIRS[N_PAIRS][2] = {   // hand
    {0,1}, {1,2}, {2,3}, {3,4},         // thumb
    {0,5}, {5,6}, {6,7}, {7,8},         // pinkie
    {0,9}, {9,10}, {10,11}, {11,12},    // middle
    {0,13}, {13,14}, {14,15}, {15,16},  // ring
    {0,17}, {17,18}, {18,19}, {19,20}   // small
};



// Uses opencv OpenPose to track the hand in the image
class OpenPoseTracker : public HandTracker{
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
    runHandTracking((new OpenPoseTracker()));
    return 0;
}