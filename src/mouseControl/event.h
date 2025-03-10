#pragma once

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
        HoldLeftClickEvent();
        HoldLeftClickEvent(CGPoint clickLocation);
        void execute();
};

class ReleaseLeftClickEvent: public Event {
    private:
        CGPoint clickLocation;
    public:
        ReleaseLeftClickEvent();
        ReleaseLeftClickEvent(CGPoint clickLocation);
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
        LeftClickEvent();
        LeftClickEvent(CGPoint clickLocation);
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


