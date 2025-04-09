/**
 * For retrieving and updating the mouse location on MacOS
 */
#include <iostream>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <ApplicationServices/ApplicationServices.h>
#include "handTracker.h"
#include "handTracking.h"
#include "cam.h"
#include "fastTracker.h"
#include "calibration.h"
#include <chrono>
#include <thread>
#include <unordered_map>
#include <string>
#include "controlState.h"
#include "event.h"



// TODO: dynamically get the screen dimensions
#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1080 


#define SIMILARITY_RANGE 5 // the pixel range in which we consider two points to be the same( need to have <= this differnce in x and y to be considered the same)









class HandTrackingApplication {
    private:
        VideoStream vs;
        HandTracker* tracker;
        cv::Point prevFingerTip = cv::Point(-1,-1);
        /**
         * If the data is < SIMILARITY_RANGE pixels away from the previous data, then the data is considered to be the same
         * @param newData The new fingertip location
         * @param prevData The previous fingertip location
         */
        cv::Point correctFingerTip(const cv::Point& newData, const cv::Point& prevData){
            if(prevData.x == -1){
                return newData;
            }
            if (abs(newData.x - prevData.x) < SIMILARITY_RANGE && abs(newData.y - prevData.y) < SIMILARITY_RANGE){ // if the new data is within 3 pixels of the previous data, then we consider it to be the same
                return cv::Point(prevData.x,prevData.y);
            }else{
                return cv::Point(newData.x,newData.y);
            }
        
        }

        /**
         * Retrieves the hand data and smooths the finger tip location
         */
        HandDataOutput getHandDataFromFrame(const cv::Mat& frame, const HandTrackingState& previousTrackingState){ 
            HandDataOutput newDataOutput = tracker->getHandData(frame, previousTrackingState);
            HandData newData = newDataOutput.handData;
             
            // TODO: move this logic into prevTrackingstate and perform refinement there in the refineHandData function
            if (this->prevFingerTip.x != -1 && newData.indexFingerPosition.x != -1){// both the previous and current fingertip locations are valid
                newData.indexFingerPosition = correctFingerTip(newData.indexFingerPosition,this->prevFingerTip);
            }

            this->prevFingerTip = newData.indexFingerPosition;
            return newDataOutput;
        }
    public:
    HandTrackingApplication(VideoStream vs, HandTracker* tracker, int targetFps)
            : vs(vs), tracker(tracker) {

            }
        
        /**
         * Retrieves a frame, processes it, and updates the mouse and state ,and executes the events
         * @param state The state of the mouse and hand tracking. mutated by hand data
         */
        void runStep(ControlState& state) {

            cv::Mat frame = vs.getFrame();

            HandDataOutput data = this->getHandDataFromFrame(frame, state.handTrackingState);
            state.updateAndExecute(data); // stores the hand tracking state for the next frame and other mouse position data


            // optional display (turn this off when releasing)
            displayHandData(frame,data.handData);
            cv::imshow("Hand Tracking", frame);
            cv::waitKey(1);


        }


};


/**
 * Application class that encapsulates the hand tracking application and the virtual keyboard GUI
 */
class Application{
    private:
        HandTrackingApplication& handTrackingApp;
        ControlState& state;
        const int FRAME_TIME_MS;

    public:

        Application(HandTrackingApplication& handTrackingApp,ControlState& state,int targetFps = 10) : handTrackingApp(handTrackingApp), state(state),  FRAME_TIME_MS(1000 / targetFps) {
        }
        
        /**
         * Uses hand data to update state, then executes events.
         * 
         */
        void run() {
        while (true) {


            auto frameStart = std::chrono::steady_clock::now();  // Start time

            // update the hand tracking state and fill event queue
            handTrackingApp.runStep(state);
        
            // Calculate time taken for processing
            auto frameEnd = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(frameEnd - frameStart);

            // Sleep only for the remaining time to maintain consistent FPS
            auto sleepTime = std::chrono::milliseconds(FRAME_TIME_MS) - elapsed;
            if (sleepTime.count() > 0) {
                std::this_thread::sleep_for(sleepTime);
            }





        }


        }

        ~Application(){
        }
};      

// TODOS:
// make detection more stable (only detect no hand if last K frames have no hand)
    // stability issue 1: index finger pos flickering back and forth even though hand is relatively stable
    // stability issue 2: random clicks when moving hand (finger count drops to 0)
// decouple hand tracking from mouse control (obtuse and 1 fingers becomes 0 fingers, should not happen in hand tracking). Add obtuse flag to HandData?
// update all docs and code to move towards retracth thumb signifies click model. make thumb retraction detection more stable, s
// make left clicks more consistent, precise, and intuitive (need to research more on how to do this beyond stable clicks and smoothing for index finger pos)
// implement dragging
// record demo
// in readme or program ask users to raise their own accessibility keyboard
// make hadn tracking invariant to rotation


// optimization for gesture detection:
// ML-based gesture detection
 // - make program to generate training data using keypressses (take snapshtos of hands in differnet gesture posutiosn)
 // - train fast model on HandData, contour info, convexity info, k -curvature, etc and use it to predict gestures
 // - compare with rule based system and see which one is better 
// make smoothing feel less laggy
// optional keyboard stuff:
// raise MAC accessibility keyboard instead of gui keyboard : https://developer.apple.com/documentation/accessibility/accessibility-api
// make keyboard typing actually type into a focused text box
// make keyboard not block text box
// make keyboard come back after closing out
// TODO: move this to a proper main file
// TODO: move all the components of the main function to a composed application class and encapsulate it into a simple run function

/**
 * Main function which runs gui in main thread and mouse control in the same  thread (as gui must be in main thread)
 */
int main(){
    VideoStream vs(0);
    TrackingRect trackingBox = parseTrackingBox(TRACKING_BOX_FILE);
    HandTracker* tracker = initBestTracker();

    

    int targetFps = 10;
    HandTrackingApplication trackingApp(vs, tracker, targetFps);
    ControlState state(trackingBox.screenWidth(), trackingBox.screenHeight(), SCREEN_WIDTH, SCREEN_HEIGHT);
    Application app(trackingApp,state);
    app.run();
    return 0;
}
