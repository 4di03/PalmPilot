#pragma once
#include <queue>
#include <memory>


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
        cv::Point lastStableIndexFingerPosition = cv::Point(-1,-1); // last stable position of the index finger (minimal movement)



    public:


        ControlState(int frameWidth, int frameHeight, int screenWidth, int screenHeight);

        void resetClickState();
        void moveMouse();

        void clickMouse();



        bool isLeftClicked();



        void createKeyPressEvent(std::string key);


        /**
         * Updates the state of the control and queues the mouse  events.
         * Then exceutes those events in FIFO order, assuring that state and queue are always in sync.
         */
        void updateAndExecute(const HandData& data);

};