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


#define NROWS 4 // number of rows in the virtual keyboard
#define NCOLS 10 // number of columns in the virtual keyboard
#define KEYBOARD_WIDTH 800 // width of the virtual keyboard
#define KEYBOARD_HEIGHT 600 // height of the virtual keyboard

// TODO: dynamically get the screen dimensions
#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1080 


// Mapping of key names to macOS key codes : https://developer.apple.com/documentation/coregraphics/cgkeycode
std::unordered_map<std::string, CGKeyCode> keyMap = {
    {"1", 18}, {"2", 19}, {"3", 20}, {"4", 21}, {"5", 23},
    {"6", 22}, {"7", 26}, {"8", 28}, {"9", 25}, {"0", 29},

    {"Q", 12}, {"W", 13}, {"E", 14}, {"R", 15}, {"T", 17},
    {"Y", 16}, {"U", 32}, {"I", 34}, {"O", 31}, {"P", 35},

    {"A", 0}, {"S", 1}, {"D", 2}, {"F", 3}, {"G", 5},
    {"H", 4}, {"J", 38}, {"K", 40}, {"L", 37}, {"Back", 51},

    {"Z", 6}, {"X", 7}, {"C", 8}, {"V", 9}, {"B", 11},
    {"N", 45}, {"M", 46}, {"Space", 49}, {"Enter", 36}  // Clear is NumPad Clear
};
static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

class KeyboardView {
    private:
    GLFWwindow* window;
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


public:
    KeyboardView(){
        initKeyboard();
    }

    /**
     * Renders the virtual keyboard GUI
     * @param state The state object containging the event queue which we use to create event handlers for the keyboard that generate events and/or update the state
     */
    void renderKeyboard(ControlState& state) {
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
                    if (keys[i][j] == "Hide") {
                        state.generateHideKeyboardEvent();
                    } else {
                        state.createKeyPressEvent(new KeyPressEvent(keys[i][j]));
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
    ~KeyboardView(){
        cleanupKeyboard();
    }
};      



CGPoint getMouseLocation() {
    CGEventRef event = CGEventCreate(NULL);
    CGPoint cursor = CGEventGetLocation(event);
    CFRelease(event);
    return cursor;
}



/**
 * Interface for all events
 */
class Event {
    public:
        virtual void execute() = 0;
};

class HoldLeftClickEvent: public Event {
    public:
        void execute() {
            CGEventRef leftClickEvent = CGEventCreateMouseEvent(
                NULL, kCGEventLeftMouseDown,
                getMouseLocation(), kCGMouseButtonLeft
            );
            CGEventPost(kCGHIDEventTap, leftClickEvent);
            CFRelease(leftClickEvent);
        }
};

class ReleaseLeftClickEvent: public Event {
    public:
        void execute() {
            CGEventRef leftReleaseEvent = CGEventCreateMouseEvent(
                NULL, kCGEventLeftMouseUp,
                getMouseLocation(), kCGMouseButtonLeft
            );
            CGEventPost(kCGHIDEventTap, leftReleaseEvent);
            CFRelease(leftReleaseEvent);
        }
};

class MouseMoveEvent : public Event {
    private:
        int x;
        int y;
    public:
        MouseMoveEvent(int x, int y) : x(x), y(y) {}
        void execute() {
            CGEventRef moveEvent = CGEventCreateMouseEvent(
                NULL, kCGEventMouseMoved,
                CGPointMake(x, y), kCGMouseButtonLeft
            );
            CGEventPost(kCGHIDEventTap, moveEvent);
            CFRelease(moveEvent);

        }
};

class LeftClickEvent: public Event{
    void execute(){
        HoldLeftClickEvent().execute();
        ReleaseLeftClickEvent().execute();
    }
};

class KeyPressEvent: public Event{
    private:
        CGKeyCode keyCode;
    public:
        KeyPressEvent(CGKeyCode keyCode) : keyCode(keyCode) {}
        KeyPressEvent(std::string key) {
            if (keyMap.find(key) == keyMap.end()) {
                std::cerr << "Key not found in key map" << std::endl;
                exit(1);
            }
            keyCode = keyMap[key];
        }

        void execute() {
            CGEventRef keyDown = CGEventCreateKeyboardEvent(NULL, keyCode, true);
            CGEventRef keyUp = CGEventCreateKeyboardEvent(NULL, keyCode, false);

            CGEventPost(kCGSessionEventTap, keyDown);
            CFRelease(keyDown);


            CGEventPost(kCGSessionEventTap, keyUp);

            CFRelease(keyUp);
        }
};

class RenderKeyboardEvent: public Event{
    private:
        KeyboardView view;
    public:
        RenderKeyboardEvent(KeyboardView view){
            this->view = view;
        }
        void execute(){
            // raise the keyboard
            view.renderKeyboard();

        }
};

class HideKeyboardEvent: public Event{
    private:
        KeyboardView view;
    public:
        HideKeyboardEvent(KeyboardView view){
            this->view = view;
        }
        void execute(){
            // hide the keyboard
            view.hideKeyboard();
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
 * Executes all events in the event queue
 */
void executeEvents(std::queue<Event*>& eventQueue) {
    while (!eventQueue.empty()) {
        Event* e = eventQueue.front();
        e->execute();
        eventQueue.pop();
    }
}

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
        KeyboardView view;


        /**
         * Updates the state of the control and queues the mouse and keyboard events
         * 
         * TODO: potenialtlyh rremove event queue if its not in sycn with the state or its too complex
         */
        void updateState(const HandData& data, ControlState& state) {
            if (!data.handDetected) {
                this->resetClickState(); // we can allow left click again if hand is not detected
                this->eventQueue.push(new ReleaseLeftClickEvent());
                return;
            }

            if (data.numFingersRaised >= 5 && !this->isKeyboardRaised()) {
                std::cout << "Raising keyboard..." << std::endl;
                this->raiseKeyboard();
            } 
            else if (data.numFingersRaised == 0 && !this->isLeftClicked()) {
                std::cout << "Left Clicking Mouse" << std::endl;
                this->clickMouse();
                this->eventQueue.push(new LeftClickEvent());
            } 
            else if (data.indexFingerPosition.x != -1) {  
                int x = data.indexFingerPosition.x;
                int y = data.indexFingerPosition.y;
                std::pair<int,int> newMouseLocation = getNewMouseLocation(std::pair<int,int>(x,y),frameDims,screenDims);
                this->moveMouse();
                this->eventQueue.push(new MouseMoveEvent(newMouseLocation.first, newMouseLocation.second));
            }
            
            

            if (this->isKeyboardRaised()){
                this->eventQueue.push(new RenderKeyboardEvent(view));
            }
        }
    public:

        std::queue<Event*> eventQueue; // queue of events to be executed

        public ControlState(int frameWidth, int frameHeight, int screenWidth, int screenHeight, KeyboardView view){
            frameDims = std::pair<int,int>(frameWidth,frameHeight);
            screenDims = std::pair<int,int>(screenWidth,screenHeight);
            this->view = view;
        }

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


        void createHideKeyboardEvent(){
            this->lowerKeyboard();
            eventQueue.push(new HideKeyboardEvent(view));
        }

        void createKeyPressEvent(std::string key){
            eventQueue.push(new KeyPressEvent(key));
        }

        /**
         * Updates the state of the control and queues the mouse and keyboard events.
         * Then exceutes those events in FIFO order, assuring that state and queue are always in sync.
         */
        void updateAndExecute(const HandData& data){
            updateState(data);
            executeEvents(eventQueue);
        }



};



/**
 * Executes the mouse movement, clicks, and keyboard actions and click given hand location
 */
class Executor{
    private:
        std::pair<int,int> frameDims; // dimensions of opencv frame on which finger location is determined
        std::pair<int,int> screenDims;  // dimensions of the screen on which the mouse is moved
    public:
        Executor(int frameWidth, int frameHeight, int screenWidth, int screenHeight){
            frameDims = std::pair<int,int>(frameWidth,frameHeight);
            screenDims = std::pair<int,int>(screenWidth,screenHeight);
        }




};



class HandTrackingApplication {
    private:
        VideoStream vs;
        HandTracker* tracker;
        Executor e;
    public:
    HandTrackingApplication(VideoStream vs, HandTracker* tracker, Executor e, int targetFps)
            : vs(vs), tracker(tracker), e(e) {}

        /**
         * Retrieves a frame, processes it, and updates the mouse and keyboard state ,and executes the events
         * @param state The state of the mouse and keyboard
         */
        void runStep(ControlState& state) {

            cv::Mat frame = vs.getFrame();
            HandData data = tracker->getHandData(frame);
            state.UpdateAndExecute(data);

            // optional display (turn this off when releasing)
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
        KeyboardView view;
        const int FRAME_TIME_MS;

    public:

        Application(HandTrackingApplication handTrackingApp, KeyboardView view,  int targetFps = 10) : handTrackingApp(handTrackingApp), view(view), FRAME_TIME_MS(1000 / targetFps) {
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
