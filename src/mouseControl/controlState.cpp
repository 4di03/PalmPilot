
#include "controlState.h"
#include "event.h"



ControlState::ControlState(int frameWidth, int frameHeight, int screenWidth, int screenHeight, KeyboardView& view): view(view){
    std::cout << "L10   " << std::endl;

    frameDims = std::pair<int,int>(frameWidth,frameHeight);
    screenDims = std::pair<int,int>(screenWidth,screenHeight);
    this->view = view;
}

void ControlState::resetClickState(){
    leftClicked = false;
}

void ControlState::moveMouse(){
    // move mouse resets click, but keyboard may still be raised
    leftClicked = false;
}

void ControlState::raiseKeyboard(){
    showKeyboard = true;
    leftClicked = false;
}

void ControlState::clickMouse(){
    leftClicked = true;
}

void ControlState::lowerKeyboard(){
    printf("Lowering keyboard...\n");
    showKeyboard = false;
}

bool ControlState::isKeyboardRaised(){
    return showKeyboard;
}

bool ControlState::isLeftClicked(){
    return leftClicked;
}


void ControlState::createHideKeyboardEvent(){
    this->lowerKeyboard();
    eventQueue.push(std::make_unique<HideKeyboardEvent>(view));
}

void ControlState::createKeyPressEvent(std::string key){
    eventQueue.push(std::make_unique<KeyPressEvent>(key));
}
/**
 * Executes all events in the event queue
 */
void ControlState::executeEvents() {
    while (!eventQueue.empty()) {
        Event* e = eventQueue.front().get();
        e->execute(); 
        eventQueue.pop();
    }
}

/**
 * Updates the state of the control and queues the mouse and keyboard events.
 * Then exceutes those events in FIFO order, assuring that state and queue are always in sync.
 */
void ControlState::updateAndExecute(const HandData& data){
    updateState(data);
    this->executeEvents(); 
}

void ControlState::updateState(const HandData& data) {
    if (!data.handDetected) {
        this->resetClickState(); // we can allow left click again if hand is not detected
        // TODO: once dragging is impletned make sure there is a way to release the drag if hand is not detected
        return;
    }

    // if (data.numFingersRaised >= 5 && !this->isKeyboardRaised()) {
    //     std::cout << "Raising keyboard..." << std::endl;
    //     this->raiseKeyboard();
    // } 
    else if (data.numFingersRaised == 0 && !this->isLeftClicked()) {
        std::cout << "Left Clicking Mouse" << std::endl;
        this->clickMouse();
        this->eventQueue.push(std::make_unique<LeftClickEvent>());
    } 
    else if (data.indexFingerPosition.x != -1) {  
        int x = data.indexFingerPosition.x;
        int y = data.indexFingerPosition.y;
        std::pair<int,int> newMouseLocation = getNewMouseLocation(std::pair<int,int>(x,y),frameDims,screenDims);
        this->moveMouse();
        this->eventQueue.push(std::make_unique<MouseMoveEvent>(newMouseLocation.first, newMouseLocation.second));
    }
    
    

    if (this->isKeyboardRaised()){
        this->eventQueue.push(std::make_unique<RenderKeyboardEvent>(view, *this));
    }
}