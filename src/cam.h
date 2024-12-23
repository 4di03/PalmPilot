#include <opencv2/opencv.hpp>
#pragma once
// Class to stream video from the camera
class VideoStream{
    private:
    cv::VideoCapture cap;
    public:
    VideoStream(int cameraIndex){
        cap = cv::VideoCapture(cameraIndex);
        // Check if the camera opened successfully
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open webcam." << std::endl;
            exit(1);
        }

    }
    // Get the next frame from the camera, updating the video stream
    cv::Mat getFrame(){
        cv::Mat frame;
        cap >> frame;
        return frame;
    }

    // Release upon destruction
    ~VideoStream(){
        cap.release();
    }
};

// TODO: handle the case where camera cannot capture frames