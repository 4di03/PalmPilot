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


ReleaseLeftClickEvent::ReleaseLeftClickEvent(): clickLocation(getMouseLocation()) {}
ReleaseLeftClickEvent::ReleaseLeftClickEvent(CGPoint clickLocation) : clickLocation(clickLocation) {}

HoldLeftClickEvent::HoldLeftClickEvent(): clickLocation(getMouseLocation()) {} // click at current mouse location if no location is provided
HoldLeftClickEvent::HoldLeftClickEvent(CGPoint clickLocation) : clickLocation(clickLocation) {}


LeftClickEvent::LeftClickEvent(): clickLocation(getMouseLocation()) {}
LeftClickEvent::LeftClickEvent(CGPoint clickLocation) : clickLocation(clickLocation) {}



