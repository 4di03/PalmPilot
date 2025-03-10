#pragma once
#include <queue>
#include "keyboardView.h"
#include <memory>


class Event;  // Forward declaration

/**
 *  Controller for the state of the mouse and keyboard, creating events and holding them in a queue.
 */
class ControlState{
    private:
        bool leftClicked = false;
        bool showKeyboard = false; // shared across threads to manage keyboard gui
        
        // used for creating mouse move events
        std::pair<int,int> frameDims; // dimensions of opencv frame on which finger location is determined
        std::pair<int,int> screenDims;  // dimensions of the screen on which the mouse is moved

        // use this for creating keyboard raise and lower events
        KeyboardView& view;
        
        std::queue<cv::Point> mousePositions; // queue of last 10 mouse positions to keep track of stability
        cv::Point lastStableIndexFingerPosition = cv::Point(-1,-1); // last stable position of the index finger (minimal movement)


        /**
         * Updates the state of the control and queues the mouse and keyboard events
         * 
         * TODO: potenialtlyh rremove event queue if its not in sycn with the state or its too complex
         */
        void updateState(const HandData& data);
    public:


        ControlState(int frameWidth, int frameHeight, int screenWidth, int screenHeight, KeyboardView& view);

        void resetClickState();
        void moveMouse();
        void raiseKeyboard();

        void clickMouse();

        void lowerKeyboard();

        bool isKeyboardRaised();

        bool isLeftClicked();


        void createHideKeyboardEvent();

        void createKeyPressEvent(std::string key);


        /**
         * Updates the state of the control and queues the mouse and keyboard events.
         * Then exceutes those events in FIFO order, assuring that state and queue are always in sync.
         */
        void updateAndExecute(const HandData& data);

};