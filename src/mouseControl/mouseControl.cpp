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
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <thread>
#include <unordered_map>
#include <string>
#include "keyboardView.h"
#include "controlState.h"
#include "event.h"



// TODO: dynamically get the screen dimensions
#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1080 














class HandTrackingApplication {
    private:
        VideoStream vs;
        HandTracker* tracker;
    public:
    HandTrackingApplication(VideoStream vs, HandTracker* tracker, int targetFps)
            : vs(vs), tracker(tracker) {}

        /**
         * Retrieves a frame, processes it, and updates the mouse and keyboard state ,and executes the events
         * @param state The state of the mouse and keyboard
         */
        void runStep(ControlState& state) {

            cv::Mat frame = vs.getFrame();
            HandData data = tracker->getHandData(frame);
            state.updateAndExecute(data);

            // optional display (turn this off when releasing)
            displayHandData(frame,data);
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
        KeyboardView& view;
        const int FRAME_TIME_MS;

    public:

        Application(HandTrackingApplication& handTrackingApp,ControlState& state, KeyboardView& view,  int targetFps = 10) : handTrackingApp(handTrackingApp), state(state), view(view), FRAME_TIME_MS(1000 / targetFps) {
        }
        
        /**
         * Uses hand data to update state, then executes events.
         * 
         * use event based system to decouple model (hand tracking) from view (keyboard gui)
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
            view.cleanupKeyboard();
        }
};      

// TODOS:
// improve click precision (since finger tip drops when the palm is closed)
// implement dragging
// make keyboard come back after closing out
// TODO: improve  overall tracking smoothness (averaging or kalman filter) (paritucallry with preceise clicks, lot of jitter on finger tips)
// TODO: reduce sensitivity of left click (smoothness may help)
// TODO: improve pinky detection so that raise keyboard is more reliable
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
    KeyboardView view = KeyboardView();
    std::cout << "Keyboard view created" << std::endl;
    ControlState state(trackingBox.screenWidth(), trackingBox.screenHeight(), SCREEN_WIDTH, SCREEN_HEIGHT, view);
    std::cout << "Control state created" << std::endl;
    Application app(trackingApp,state, view);
    app.run();
    return 0;
}
