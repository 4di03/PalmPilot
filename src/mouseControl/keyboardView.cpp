#include "keyboardView.h"
#include "controlState.h"
#include "event.h"
static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// code taken from imgui example
void KeyboardView::initKeyboard() {
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
    std::cout << "setting platform and renderer backends" << std::endl;
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


KeyboardView::KeyboardView(){
    std::cout <<"Initializing keyboard" << std::endl;
    initKeyboard();
}

/**
 * Renders the virtual keyboard GUI
 * @param state The state object containging the event queue which we use to create event handlers for the keyboard that generate events and/or update the state
 */
void KeyboardView::renderKeyboard(ControlState& state) {
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
                if (strcmp(keys[i][j] ,"Hide") == 0) {
                    state.createHideKeyboardEvent();
                } else {
                    state.createKeyPressEvent(keys[i][j]);
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

void KeyboardView::hideKeyboard() {
    glfwHideWindow(window);
}

void KeyboardView::cleanupKeyboard(){

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    if (window) {
        glfwDestroyWindow(window);
        window = nullptr;
    }


    glfwTerminate();
}
KeyboardView::~KeyboardView(){
    cleanupKeyboard();
}

