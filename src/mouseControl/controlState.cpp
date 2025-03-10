
#include "controlState.h"
#include "event.h"

ControlState::ControlState(int frameWidth, int frameHeight, int screenWidth, int screenHeight){
    std::cout << "L10   " << std::endl;

    frameDims = std::pair<int,int>(frameWidth,frameHeight);
    screenDims = std::pair<int,int>(screenWidth,screenHeight);
}

void ControlState::resetClickState(){
    leftClicked = false;
}

void ControlState::moveMouse(int x, int y){
    leftClicked = false;

    this->mousePositions.push(cv::Point(x,y));
    if (this->mousePositions.size() > MOUSE_POS_QUEUE_SIZE){
        this->mousePositions.pop();
    }

    
}



void ControlState::clickMouse(){
    leftClicked = true;
}





bool ControlState::isLeftClicked(){
    return leftClicked;
}



/**
 * Returns the last stable click location if it exists, else returns (-1,-1)
 * A click location is considered stable if the last 3 consecutive points are within 3 pixels of each other
 */
CGPoint ControlState::getLastStableClickLocation(){
    
    // get most recent sequence of values that are similar within (3 pixels vertically and horizontally)

    // get copy of the last 10 mouse positions
    std::queue<cv::Point> mousePositionsCopy = this->mousePositions;

    // go from the back and try and find a sequence of at least 3 points that are within 3 pixels of each other
    int consecutivePoints = 1;
    cv::Point prevPoint = mousePositionsCopy.front();
    mousePositionsCopy.pop();
    while (consecutivePoints < STABLE_CONSECUTIVE_POINTS && !mousePositionsCopy.empty()){
        cv::Point currentPoint = mousePositionsCopy.front();
        mousePositionsCopy.pop();
        if (abs(currentPoint.x - prevPoint.x) <= 3 && abs(currentPoint.y - prevPoint.y) <= 3){
            consecutivePoints++;
        }else{
            consecutivePoints = 0;
        }
        
    }

    // if we found a sequence of 3 consecutive points that are stable, return the last point in the sequence
    if (consecutivePoints == STABLE_CONSECUTIVE_POINTS){
        return CGPointMake(prevPoint.x,prevPoint.y);
    }else{
        return CGPointMake(-1,-1);
    }
    

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
        // TODO: cache last stable click location if needed
        LeftClickEvent(this->getLastStableClickLocation()).execute();
    } 
    else if (data.indexFingerPosition.x != -1) {  
        int x = data.indexFingerPosition.x;
        int y = data.indexFingerPosition.y;
        std::pair<int,int> newMouseLocation = getNewMouseLocation(std::pair<int,int>(x,y),frameDims,screenDims);
        this->moveMouse(x,y);
        MouseMoveEvent(newMouseLocation.first, newMouseLocation.second).execute();
    }
    
    


}