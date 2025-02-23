#pragma once  // Prevents recursive inclusion

#include <iostream>
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <thread>
#include <atomic>

/**
 * Virtual Keyboard Class using ImGui (Runs in a Separate Thread)
 */
class VirtualKeyboard {
public:
    VirtualKeyboard();
    ~VirtualKeyboard();

    void show();
    void stop();
    bool isRunning() const;

private:
    GLFWwindow* window;
    std::atomic<bool> running;
    std::thread keyboardThread;

    void runKeyboardLoop();
};
