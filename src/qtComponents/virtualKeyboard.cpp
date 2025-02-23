#include "virtualKeyboard.h"
#include <dispatch/dispatch.h>


/**
 * Virtual Keyboard Constructor
 */
VirtualKeyboard::VirtualKeyboard() {
    running = false;
}

/**
 * Show the virtual keyboard (non-blocking, runs in a separate thread)
 */
void VirtualKeyboard::show() {
    if (running) return;  // Already running

    running = true;
    keyboardThread = std::thread(&VirtualKeyboard::runKeyboardLoop, this);
}

/**
 * Stops the keyboard thread
 */
void VirtualKeyboard::stop() {
    running = false;
    if (keyboardThread.joinable()) {
        keyboardThread.join();
    }
}

/**
 * Check if the keyboard is running
 */
bool VirtualKeyboard::isRunning() const {
    return running;
}

// TODO: rearchitect main loop so that gui is in main thread and mouse control + tracking is in a separate thread
// do some reserach on how this is usually done (ask gpt)
/**
 * Runs the keyboard UI in a separate thread
 */
void VirtualKeyboard::runKeyboardLoop() {
    // Initialize GLFW
    std::cout << "Hello from keyboard loop" << std::endl;
    if (!glfwInit()) {
        std::cerr << "GLFW initialization failed" << std::endl;
        running = false;
        return;
    }
    std::cout << "Initialized GLFW" << std::endl;

    window = glfwCreateWindow(500, 300, "Virtual Keyboard", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        running = false;
        return;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    while (running && !glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Virtual Keyboard");
        static std::string text;

        const char* keys[4][10] = {
            {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0"},
            {"Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P"},
            {"A", "S", "D", "F", "G", "H", "J", "K", "L", "Back"},
            {"Z", "X", "C", "V", "B", "N", "M", "Space", "Enter", "Clear"}
        };

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 10; j++) {
                if (ImGui::Button(keys[i][j], ImVec2(40, 40))) {
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

        ImGui::Text("Output: %s", text.c_str());
        ImGui::End();

        ImGui::Render();
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}


/**
 * Destructor
 */
VirtualKeyboard::~VirtualKeyboard() {
    stop();
}