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
#define NROWS 4 // number of rows in the virtual keyboard
#define NCOLS 10 // number of columns in the virtual keyboard
#define KEYBOARD_WIDTH 800 // width of the virtual keyboard
#define KEYBOARD_HEIGHT 600 // height of the virtual keyboard

// TODO: dynamically get the screen dimensions
#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1080 
static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}
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
 * State of FSM for controlling the mouse and keyboard
 */
class ControlState{
    private:
        bool leftClicked = false;
        bool showKeyboard = false; // shared across threads to manage keyboard gui
    public:

        void resetClickState(){
            leftClicked = false;
        }

        void moveMouse(){
            // move mouse resets click, but keyboard may still be raised
            leftClicked = false;
        }

        void raiseKeyboard(){
            showKeyboard = true;
            leftClicked = false;
        }

        void clickMouse(){
            leftClicked = true;
        }

        void lowerKeyboard(){
            printf("Lowering keyboard...\n");
            showKeyboard = false;
        }

        bool isKeyboardRaised(){
            return showKeyboard;
        }

        bool isLeftClicked(){
            return leftClicked;
        }


};

class KeyboardController{

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

        /**
         * Executes the mouse movement, clicks, and keyboard raising and click given hand location
         * Updates the state of the control
         */
        void execute(const HandData& data, ControlState& state) {
            if (!data.handDetected) {
                state.resetClickState(); // we can allow left click again if hand is not detected
                return;
            }

            if (data.numFingersRaised >= 5 && !state.isKeyboardRaised()) {
                std::cout << "Raising keyboard..." << std::endl;
                state.raiseKeyboard();
            } 
            else if (data.numFingersRaised == 0 && !state.isLeftClicked()) {
                std::cout << "Left Clicking Mouse" << std::endl;
                state.clickMouse();
                mc.leftClick();
            } 
            else if (data.indexFingerPosition.x != -1) {  
                int x = data.indexFingerPosition.x;
                int y = data.indexFingerPosition.y;


                std::pair<int,int> newMouseLocation = getNewMouseLocation(std::pair<int,int>(x,y),frameDims,screenDims);
                mc.moveMouseToAbsolute(newMouseLocation.first, newMouseLocation.second);
                state.moveMouse();
            }   
        }



};

//


class HandTrackingApplication {
    private:
        VideoStream vs;
        HandTracker* tracker;
        Executor e;
    public:
    HandTrackingApplication(VideoStream vs, HandTracker* tracker, Executor e, int targetFps)
            : vs(vs), tracker(tracker), e(e) {}

        /**
         * Retrieves a frame, processes it, and updates the mouse and keyboard state.
         * @param state The state of the mouse and keyboard
         */
        void runStep(ControlState& state) {

            cv::Mat frame = vs.getFrame();
            HandData data = tracker->getHandData(frame);
            e.execute(data,state);

            // optional display
            displayHandData(frame,data);
            cv::imshow("Hand Tracking", frame);
            cv::waitKey(1);

        }

    executeKeyboardCommand(std::string key){

    }
};

/**
 * Application class that encapsulates the hand tracking application and the virtual keyboard GUI
 */
class Application{
    private:
        HandTrackingApplication handTrackingApp;
        ControlState state;
        GLFWwindow* window;
        const int FRAME_TIME_MS;

    public:
        // code taken from imgui example
        void initKeyboard() {
            glfwSetErrorCallback(glfw_error_callback);
            if (!glfwInit()){
                std::cerr << "Failed to initialize GLFW" << std::endl;
                exit(1);
            }
        
            // Decide GL+GLSL versions
        #if defined(IMGUI_IMPL_OPENGL_ES2)
            // GL ES 2.0 + GLSL 100 (WebGL 1.0)
            const char* glsl_version = "#version 100";
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
            glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
        #elif defined(IMGUI_IMPL_OPENGL_ES3)
            // GL ES 3.0 + GLSL 300 es (WebGL 2.0)
            const char* glsl_version = "#version 300 es";
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
            glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
        #elif defined(__APPLE__)
            // GL 3.2 + GLSL 150
            const char* glsl_version = "#version 150";
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
        #else
            // GL 3.0 + GLSL 130
            const char* glsl_version = "#version 130";
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
            //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
            //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
        #endif
        
            // Create window with graphics context
            window = glfwCreateWindow(KEYBOARD_WIDTH, KEYBOARD_HEIGHT + 10, "Keyboard", nullptr, nullptr);
            if (window == nullptr){
                std::cerr << "Failed to create GLFW window" << std::endl;
                glfwTerminate();
                exit(1);
            }
            glfwMakeContextCurrent(window);
            glfwSwapInterval(1); // Enable vsync
        
            // Setup Dear ImGui context
            IMGUI_CHECKVERSION();
            ImGui::CreateContext();

            // Setup Dear ImGui style
            ImGui::StyleColorsDark();
            //ImGui::StyleColorsLight();
        
            // Setup Platform/Renderer backends
            ImGui_ImplGlfw_InitForOpenGL(window, true);
        #ifdef __EMSCRIPTEN__
            ImGui_ImplGlfw_InstallEmscriptenCallbacks(window, "#canvas");
        #endif
            ImGui_ImplOpenGL3_Init(glsl_version);
        
            // Our state
            bool show_demo_window = true;
            bool show_another_window = false;
            ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
        
            hideKeyboard(); // hide the keyboard initially
        }

        Application(HandTrackingApplication handTrackingApp, int targetFps = 10) : handTrackingApp(handTrackingApp), FRAME_TIME_MS(1000 / targetFps) {
            initKeyboard(); 
        }


 

        void cleanupKeyboard(){

            ImGui_ImplOpenGL3_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
            if (window) {
                glfwDestroyWindow(window);
                window = nullptr;
            }
        

            glfwTerminate();
        }

        /**
         * Renders the virtual keyboard GUI
         */
        void renderKeyboard(){
            if (!window) {
                std:: cerr << "Window is null" << std::endl;
                exit(1);
            }

            glfwShowWindow(window); // unhides the window if it was hidden




            glfwPollEvents();
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            // begin but remove padding
            ImGui::SetNextWindowPos(ImVec2(0, 0));
            ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
            ImGui::Begin("Keyboard", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
            
            static std::string text;
            const char* keys[NROWS][NCOLS] = {
                {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0"},
                {"Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P"},
                {"A", "S", "D", "F", "G", "H", "J", "K", "L", "Back"},
                {"Z", "X", "C", "V", "B", "N", "M", "Space", "Enter", "Clear"}
            };

            int frameWidth= KEYBOARD_WIDTH;
            int frameHeight = KEYBOARD_HEIGHT;

            float buttonWidth = frameWidth / NCOLS;
            float buttonHeight = frameHeight / (NROWS );

            for (int i = 0; i < NROWS; i++) {
                for (int j = 0; j < NCOLS; j++) {
                    if (ImGui::Button(keys[i][j], ImVec2(buttonWidth, buttonHeight))) {
                        if (strcmp(keys[i][j], "Back") == 0 && !text.empty()) {
                            text.pop_back();
                        } else if (strcmp(keys[i][j], "Clear") == 0) {
                            text.clear();
                        } else if (strcmp(keys[i][j], "Space") == 0) {
                            text += " ";
                        } else if (strcmp(keys[i][j], "Enter") == 0) {
                            text += "\n";
                        } else {
                            text += keys[i][j];
                        }
                    }
                    ImGui::SameLine();
                }
                ImGui::NewLine();
            }

            ImGui::End();

            ImGui::Render();
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window);

        }

        
        void hideKeyboard() {
            glfwHideWindow(window);
        }
        void run() {

        bool keyboardClosed = true;
        while (true) {
            auto frameStart = std::chrono::steady_clock::now();  // Start time

            // update the hand tracking state
            handTrackingApp.runStep(state);

            
            // render the keyboard if needed
            if (state.isKeyboardRaised() && !glfwWindowShouldClose(window)) {
                printf("Rendering keyboard...\n");
                renderKeyboard();
            }else{// make sure we only close it once for each time it is opened
                printf("Hiding keyboard...\n");
                hideKeyboard();
            }



            // Calculate time taken for processing
            auto frameEnd = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(frameEnd - frameStart);

            // Sleep only for the remaining time to maintain consistent FPS
            auto sleepTime = std::chrono::milliseconds(FRAME_TIME_MS) - elapsed;
            if (sleepTime.count() > 0) {
                std::this_thread::sleep_for(sleepTime);
            }

        }
        cleanupKeyboard();


        }
        ~Application(){
            cleanupKeyboard();
        }
};      



// TODO: make raisekeyboard actually bring up a keyboard GUI that can input keys.
// make keyboard come back after closing out
// make keybaord not blocack
// TODO: improve smoothness (averaging or kalman filter) (paritucallry with preceise clicks, lot of jitter on finger tips)
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
    Executor e(trackingBox.screenWidth(), trackingBox.screenHeight(), SCREEN_WIDTH, SCREEN_HEIGHT);

    

    int targetFps = 10;
    HandTrackingApplication trackingApp(vs, tracker, e, targetFps);
    Application app(trackingApp);
    app.run();
    return 0;
}
