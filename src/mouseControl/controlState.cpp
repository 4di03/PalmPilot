
#include "controlState.h"
#include "event.h"



ControlState::ControlState(int frameWidth, int frameHeight, int screenWidth, int screenHeight): view(view){
    std::cout << "L10   " << std::endl;

    frameDims = std::pair<int,int>(frameWidth,frameHeight);
    screenDims = std::pair<int,int>(screenWidth,screenHeight);
    this->view = view;
}

void ControlState::resetClickState(){
    leftClicked = false;
}

void ControlState::moveMouse(){
    leftClicked = false;

    
}



void ControlState::clickMouse(){
    leftClicked = true;
}





bool ControlState::isLeftClicked(){
    return leftClicked;
}




void ControlState::createKeyPressEvent(std::string key){
    eventQueue.push(std::make_unique<KeyPressEvent>(key));
}

/**
 * Updates the state of the control
 * Then exceutes the necessary event on the mouse
 */
void ControlState::updateAndExecute(const HandData& data) {
    if (!data.handDetected) {
        this->resetClickState(); // we can allow left click again if hand is not detected
        // TODO: once dragging is impletned make sure there is a way to release the drag if hand is not detected
        return;
    }

    if (data.numFingersRaised == 0 && !this->isLeftClicked()) {
        std::cout << "Left Clicking Mouse" << std::endl;
        this->clickMouse();
        LeftClickEvent(data->lastStableIndexFingerPosition).execute();
    } 
    else if (data.indexFingerPosition.x != -1) {  
        int x = data.indexFingerPosition.x;
        int y = data.indexFingerPosition.y;
        std::pair<int,int> newMouseLocation = getNewMouseLocation(std::pair<int,int>(x,y),frameDims,screenDims);
        this->moveMouse();
        MouseMoveEvent(newMouseLocation.first, newMouseLocation.second).execute();
    }
    
    


}