#include <opencv2/opencv.hpp>
#pragma once
#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720
// Class to stream video from the camera
class VideoStream{
    private:
    cv::VideoCapture cap;
    cv::Size frameSize; // size that the frame is resized to, first value is width, second is height
    public:

    VideoStream(int cameraIndex, int frameWidth = FRAME_WIDTH, int frameHeight = FRAME_HEIGHT){
        this->cap = cv::VideoCapture(cameraIndex);
        // Check if the camera opened successfully
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open webcam." << std::endl;
            exit(1);
        }

        this->frameSize = cv::Size(frameWidth,frameHeight);

    }

    cv::Size getFrameSize(){
        return this->frameSize;
    }

    // Get the next frame from the camera, updating the video stream
    cv::Mat getFrame(){
        cv::Mat frame;
        cap >> frame;
        cv::resize(frame,frame,this->frameSize);
        return frame;
    }

    // Release upon destruction
    ~VideoStream(){
        std::cout << "Releasing camera." << std::endl;
        cap.release();
    }
};

// TODO: handle the case where camera cannot capture frames