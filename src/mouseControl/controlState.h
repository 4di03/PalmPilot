#pragma once
#include <memory>
#include <opencv2/opencv.hpp>
#include <CoreGraphics/CoreGraphics.h>
#include "handTracker.h"
#define MOUSE_POS_QUEUE_SIZE 10 // number of mouse positions to keep track of
#define STABLE_CONSECUTIVE_POINTS 3 // number of consecutive points that are considered stable

class Event;  // Forward declaration

/**
 *  Controller for the state of the mouse, creating events and holding them in a queue.
 */
class ControlState{
    private:
        bool leftClicked = false;
        
        // used for creating mouse move events
        std::pair<int,int> frameDims; // dimensions of opencv frame on which finger location is determined
        std::pair<int,int> screenDims;  // dimensions of the screen on which the mouse is moved

        std::queue<cv::Point> mousePositions; // queue of last 10 mouse positions to keep track of stability



    public:


        ControlState(int frameWidth, int frameHeight, int screenWidth, int screenHeight);

        CGPoint getLastStableClickLocation();
        void resetClickState();
        void moveMouse(int x, int y);

        void clickMouse();



        bool isLeftClicked();



        void createKeyPressEvent(std::string key);


        /**
         * Updates the state of the control and queues the mouse  events.
         * Then exceutes those events in FIFO order, assuring that state and queue are always in sync.
         */
        void updateAndExecute(const HandData& data);

};