/**
 * For retrieving and updating the mouse location on MacOS
 */
#include <iostream>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <ApplicationServices/ApplicationServices.h>
#include "handTracker.h"
#include "cam.h"
#include "fastTracker.h"
#include <chrono>
#include <thread>


// TODO: dynamically get the screen dimensions
#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1080 

CGPoint getMouseLocation() {
    CGEventRef event = CGEventCreate(NULL);
    CGPoint cursor = CGEventGetLocation(event);
    CFRelease(event);
    return cursor;
}

class MouseController{
    private:
        CGPoint mousePoint;

    public:
        MouseController()
        {
            mousePoint = getMouseLocation();

        }

        int getMouseX(){
            return mousePoint.x;
        }
        int getMouseY(){
            return mousePoint.y;
        }
        
        /**
         * Moves the mouse to the absolute location (x,y)
         * @param x The x-coordinate of the new mouse location (column)
         * @param y The y-coordinate of the new mouse location (row)
         * For example, (100,250) would move the mouse to the pixel 100th column and 250th row from the top left corner
         */
        void moveMouseToAbsolute(int x, int y){
            // Set the new mouse location
            mousePoint.x = x;
            mousePoint.y = y;

            // Move the mouse
            CGEventRef moveEvent = CGEventCreateMouseEvent(
                NULL, kCGEventMouseMoved,
                mousePoint, kCGMouseButtonLeft
            );
            CGEventPost(kCGHIDEventTap, moveEvent);
            CFRelease(moveEvent);
            return;
        }

        void moveMouse(int deltaX,int deltaY){
            this->moveMouseToAbsolute(mousePoint.x + deltaX, mousePoint.y + deltaY);
        }

        void leftClick() const{

        // Simulate a left mouse button click
        CGEventRef leftClickEvent = CGEventCreateMouseEvent(
            NULL, kCGEventLeftMouseDown,
            mousePoint, kCGMouseButtonLeft
        );
        CGEventPost(kCGHIDEventTap, leftClickEvent);
        CFRelease(leftClickEvent);

        // Release the left mouse button
        CGEventRef leftReleaseEvent = CGEventCreateMouseEvent(
            NULL, kCGEventLeftMouseUp,
            mousePoint, kCGMouseButtonLeft
        );
        CGEventPost(kCGHIDEventTap, leftReleaseEvent);
        CFRelease(leftReleaseEvent);
        return;

        }
};

/**
 * Calculates the new mouse location (x,y) given the finger location (x,y), frame dimensions, and screen dimensions
 */
std::pair<int,int> getNewMouseLocation(std::pair<int,int> fingerLoc,std::pair<int,int> frameDims, std::pair<int,int> screenDims){

    int x = fingerLoc.first;
    int y = fingerLoc.second;

    int frameWidth = frameDims.first;
    int frameHeight = frameDims.second;

    int screenWidth = screenDims.first;
    int screenHeight = screenDims.second;

    // can be optimzied to just pass a scaling factor to avoid recalculating this every time

    int newX = static_cast<int>((static_cast<float>(x) / frameWidth) * screenWidth);
    int newY = static_cast<int>((static_cast<float>(y) / frameHeight) * screenHeight);
    


    // invert the x and y as y is row and x is column
    return std::pair<int,int>(newX,newY);


}


/**
 * Raises the keyboard
 */
void raiseKeyboard(){
    std::cout << "Keyboard Raised" << std::endl;
}

/**
 * Executes the mouse movement, clicks, and keyboard actions and click given hand location
 */
class Executor{
    private:
        MouseController mc = MouseController(); // instance-specific mouse controller
        std::pair<int,int> frameDims; // dimensions of opencv frame on which finger location is determined
        std::pair<int,int> screenDims;  // dimensions of the screen on which the mouse is moved
    public:
        Executor(int frameWidth, int frameHeight, int screenWidth, int screenHeight){
            frameDims = std::pair<int,int>(frameWidth,frameHeight);
            screenDims = std::pair<int,int>(screenWidth,screenHeight);
        }

        void execute(const HandData& data){
            


            if (data.handDetected){
                if (data.numFingersRaised >= 5){
                    raiseKeyboard();
                }else if (data.indexFingerPosition != NULL_POINT){ // if the index finger is raised and not all 5 fingers are raised
                    // Move the mouse to the index finger position

                    int x = data.indexFingerPosition.x;
                    int y = data.indexFingerPosition.y;

                    std::cout << "Found finger at: " << x << "," << y << std::endl;

                    std::pair<int,int> newMouseLocation = getNewMouseLocation(std::pair<int,int>(x,y),frameDims,screenDims);
                    
                    std::cout << "Moving Mouse to: " << newMouseLocation.first << "," << newMouseLocation.second << std::endl;
                    mc.moveMouseToAbsolute(newMouseLocation.first, newMouseLocation.second);
                }else{
                    // if hand is raised but no fingers are raised, click the mouse
                    std::cout << "Left Clicking Mouse" << std::endl;
                    mc.leftClick();
                }
            }else{
                std::cout << "No hand detected, not doing anything" << std::endl;
            }
        }
};



// TODO: move this to a proper main file
// TODO: move all the components of the main function to a composed application class and encapsulate it into a simple run function
int main() {
    VideoStream vs(0);
    cv::Size frameSize = vs.getFrameSize();
    HandTracker* tracker = initBestTracker();
    Executor e(frameSize.width, frameSize.height, SCREEN_WIDTH, SCREEN_HEIGHT);

    int targetFps = 30;
    const int FRAME_TIME_MS = 1000/targetFps;  // Time in milliseconds for each frame
    
    while (true) {
        auto frameStart = std::chrono::steady_clock::now();  // Start time

        cv::Mat frame = vs.getFrame();
        HandData data = tracker->getHandData(frame);
        e.execute(data);

        // Calculate time taken for processing
        auto frameEnd = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(frameEnd - frameStart);

        // Sleep only for the remaining time to maintain consistent FPS
        auto sleepTime = std::chrono::milliseconds(FRAME_TIME_MS) - elapsed;
        if (sleepTime.count() > 0) {
            std::this_thread::sleep_for(sleepTime);
        }
    }
    return 0;
}