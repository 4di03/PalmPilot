#include "event.h"
CGPoint getMouseLocation() {
    CGEventRef event = CGEventCreate(NULL);
    CGPoint cursor = CGEventGetLocation(event);
    CFRelease(event);
    return cursor;
}


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

void HoldLeftClickEvent::execute() {
    CGEventRef leftClickEvent = CGEventCreateMouseEvent(
        NULL, kCGEventLeftMouseDown,
        getMouseLocation(), kCGMouseButtonLeft
    );
    CGEventPost(kCGHIDEventTap, leftClickEvent);
    CFRelease(leftClickEvent);
}

void ReleaseLeftClickEvent::execute() {
    CGEventRef leftReleaseEvent = CGEventCreateMouseEvent(
        NULL, kCGEventLeftMouseUp,
        getMouseLocation(), kCGMouseButtonLeft
    );
    CGEventPost(kCGHIDEventTap, leftReleaseEvent);
    CFRelease(leftReleaseEvent);
}


void MouseMoveEvent::execute() {
    CGEventRef moveEvent = CGEventCreateMouseEvent(
        NULL, kCGEventMouseMoved,
        CGPointMake(x, y), kCGMouseButtonLeft
    );
    CGEventPost(kCGHIDEventTap, moveEvent);
    CFRelease(moveEvent);

}

void LeftClickEvent::execute(){
    HoldLeftClickEvent(this->clickLocation).execute();
    ReleaseLeftClickEvent(this->clickLocation).execute();
}

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


KeyPressEvent::KeyPressEvent(std::string key) {
        if (keyMap.find(key) == keyMap.end()) {
            std::cerr << "Key not found in key map" << std::endl;
            exit(1);
        }
        keyCode = keyMap[key];
    }
    

void KeyPressEvent::execute() {

    CGEventRef keyDown = CGEventCreateKeyboardEvent(NULL, keyCode, true);
    CGEventRef keyUp = CGEventCreateKeyboardEvent(NULL, keyCode, false);

    CGEventPost(kCGSessionEventTap, keyDown);
    CFRelease(keyDown);


    CGEventPost(kCGSessionEventTap, keyUp);

    CFRelease(keyUp);
}


void RenderKeyboardEvent::execute(){
    // raise the keyboard
    view.renderKeyboard(state);

}

void HideKeyboardEvent::execute(){
    // hide the keyboard
    view.hideKeyboard();
}
