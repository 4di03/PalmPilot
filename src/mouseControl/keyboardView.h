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

#pragma once

class ControlState; // forward declaration

class KeyboardView {
    private:
    GLFWwindow* window;
    void initKeyboard();
  


public:
    KeyboardView();
    /**
     * Renders the virtual keyboard GUI
     * @param state The state object containging the event queue which we use to create event handlers for the keyboard that generate events and/or update the state
     */
    void renderKeyboard(ControlState& state);
    void hideKeyboard();
    void cleanupKeyboard();
    ~KeyboardView();
};      
