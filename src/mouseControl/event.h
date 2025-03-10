#pragma once

#include "keyboardView.h"
#include "controlState.h"

std::pair<int,int> getNewMouseLocation(std::pair<int,int> fingerLoc,std::pair<int,int> frameDims, std::pair<int,int> screenDims);
/**
 * Interface for all events
 */
class Event {
    public:
        virtual void execute() = 0;
        virtual ~Event() {}  // Virtual destructor (fixes warning)

};


class HoldLeftClickEvent: public Event {
    private:
        CGPoint clickLocation;
    public:
        HoldLeftClickEvent(): clickLocation(getMouseLocation()) {} // click at current mouse location if no location is provided
        HoldLeftClickEvent(CGPoint clickLocation) : clickLocation(clickLocation) {}
        void execute();
};

class ReleaseLeftClickEvent: public Event {
    private:
        CGPoint clickLocation;
    public:
        ReleaseLeftClickEvent(): clickLocation(getMouseLocation()) {}
        ReleaseLeftClickEvent(CGPoint clickLocation) : clickLocation(clickLocation) {}
        void execute();
};

class MouseMoveEvent : public Event {
    private:
        int x;
        int y;
    public:
        MouseMoveEvent(int x, int y) : x(x), y(y) {}
        void execute();
};

class LeftClickEvent: public Event{
    private:
        CGPoint clickLocation;
    public:
        LeftClickEvent(): clickLocation(getMouseLocation()) {}
        LeftClickEvent(CGPoint clickLocation) : clickLocation(clickLocation) {}
        void execute();
};

class KeyPressEvent: public Event{
    private:
        CGKeyCode keyCode;
    public:
        KeyPressEvent(CGKeyCode keyCode) : keyCode(keyCode) {}
        KeyPressEvent(std::string key);

        void execute();
};

class RenderKeyboardEvent: public Event{
    private:
        KeyboardView& view;
        ControlState& state;
    public:
        RenderKeyboardEvent(KeyboardView& view, ControlState& state) : view(view), state(state) {}
        void execute();
};

class HideKeyboardEvent: public Event{
    private:
        KeyboardView& view;
    public:
    HideKeyboardEvent(KeyboardView& view) 
    : view(view) {  // ✅ Properly initialize the reference member
}
    void execute();
};