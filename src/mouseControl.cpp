#include <iostream>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <ApplicationServices/ApplicationServices.h>
using namespace std;


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
        
        void moveMouse(int deltaX,int deltaY){
            // Set the new mouse location
            mousePoint.x += deltaX;
            mousePoint.y += deltaY;

            // Move the mouse
            CGEventRef moveEvent = CGEventCreateMouseEvent(
                NULL, kCGEventMouseMoved,
                mousePoint, kCGHIDEventTap
            );
            CGEventPost(kCGHIDEventTap, moveEvent);
            CFRelease(moveEvent);
            return;
        }

        void leftClick(){

        // Simulate a left mouse button click
        CGEventRef leftClickEvent = CGEventCreateMouseEvent(
            NULL, kCGEventLeftMouseDown,
            mousePoint, kCGHIDEventTap
        );
        CGEventPost(kCGHIDEventTap, leftClickEvent);
        CFRelease(leftClickEvent);

        // Release the left mouse button
        CGEventRef leftReleaseEvent = CGEventCreateMouseEvent(
            NULL, kCGEventLeftMouseUp,
            mousePoint, kCGHIDEventTap
        );
        CGEventPost(kCGHIDEventTap, leftReleaseEvent);
        CFRelease(leftReleaseEvent);
        return;

        }
};

int main() {

    // Get the current mouse location
    int ct;
    while (true){
        cout << "Mouse Location: " << getMouseLocation().x << ", " << getMouseLocation().y << endl;
    // if (ct %100 == 0){
    // MouseController mc = MouseController();
    // // Set the new mouse location
    // mc.moveMouse(100,-100);
    // mc.leftClick();
    // ct = 0;
    // }
    ct++;

    } 

    return 0;
}
// sample usage: 
// clang++  src/mouseControl.cpp -framework CoreGraphics -framework CoreFoundation -o mouseControl && ./mouseControl
 