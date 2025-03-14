
#include "controlState.h"
#include "event.h"

ControlState::ControlState(int frameWidth, int frameHeight, int screenWidth, int screenHeight){

    frameDims = std::pair<int,int>(frameWidth,frameHeight);
    screenDims = std::pair<int,int>(screenWidth,screenHeight);
}

void ControlState::resetClickState(){
    leftClicked = false;
}

void ControlState::moveMouse(int x, int y){
    leftClicked = false;

    this->mousePositions.push_back(cv::Point(x,y));
    if (this->mousePositions.size() > MOUSE_POS_QUEUE_SIZE){
        this->mousePositions.pop_front();
    }

    
}



void ControlState::clickMouse(){
    leftClicked = true;
}





bool ControlState::isLeftClicked(){
    return leftClicked;
}

void printQueue(std::deque<cv::Point> q){
    printf("[ ");
    while (!q.empty()){
        cv::Point p = q.front();
        printf("(%d,%d) ", p.x, p.y);
        q.pop_front();
    }
    printf("]\n");

}

/**
 * Returns the last stable click location if it exists, else returns (-1,-1)
 * A click location is considered stable if the last 3 consecutive points are within 3 pixels of each other
 */
CGPoint ControlState::getLastStableClickLocation(){
    if (this->mousePositions.size() < STABLE_CONSECUTIVE_POINTS){
        return CGPointMake(-1,-1);
    }
    // get most recent sequence of values that are similar within (3 pixels vertically and horizontally)

    // get copy of the last 10 mouse positions
    std::deque<cv::Point> mousePositionsCopy = this->mousePositions;


    // go from the back and try and find a sequence of at least 3 points that are within 3 pixels of each other
    int consecutivePoints = 1;
    cv::Point prevPoint = mousePositionsCopy.back();
    mousePositionsCopy.pop_back();
    while (consecutivePoints < STABLE_CONSECUTIVE_POINTS && !mousePositionsCopy.empty()){
        cv::Point currentPoint = mousePositionsCopy.back();
        mousePositionsCopy.pop_back();
        if (abs(currentPoint.x - prevPoint.x) <= 3 && abs(currentPoint.y - prevPoint.y) <= 3){
            consecutivePoints++;
        }else{
            consecutivePoints = 1;
        }

        prevPoint = currentPoint;
        
    }

    // if we found a sequence of 3 consecutive points that are stable, return the last point in the sequence
    if (consecutivePoints == STABLE_CONSECUTIVE_POINTS){
        return CGPointMake(prevPoint.x,prevPoint.y);
    }else{
        printf("returning null stable click location\n");
        printQueue(this->mousePositions);
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

        // CGPoint lastStableClickLocation = this->getLastStableClickLocation();

        // if (lastStableClickLocation.x == -1) {// just use current index finger position if we can't find a stable click location
        //     lastStableClickLocation = CGPointMake(data.indexFingerPosition.x, data.indexFingerPosition.y);
        // }
        CGPoint lastStableClickLocation = CGPointMake(data.indexFingerPosition.x, data.indexFingerPosition.y);

        std::cout << "Left Clicking Mouse at :"  << lastStableClickLocation.x << " " << lastStableClickLocation.y << std::endl;
        this->clickMouse();
        // TODO: cache last stable click location if needed
        LeftClickEvent(lastStableClickLocation).execute();
    } 
    else if (data.indexFingerPosition.x != -1) {  
        int x = data.indexFingerPosition.x;
        int y = data.indexFingerPosition.y;
        std::pair<int,int> newMouseLocation = getNewMouseLocation(std::pair<int,int>(x,y),frameDims,screenDims);
        this->moveMouse(x,y);
        MouseMoveEvent(newMouseLocation.first, newMouseLocation.second).execute();
    }
    
    


}