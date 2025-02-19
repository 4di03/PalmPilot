#include "calibration.h"

int main(){
    // ask user if they want to calibrate the color range
    std::cout << "Do you want to calibrate the color range? (y/n): ";
    char response;
    std::cin >> response;
    if (response == 'y'){
        calibrateColorRange();
        colorRange c = parseColorRange(COLOR_RANGE_FILE);
        std::cout << "Saved color range: \n" << c.toString() << std::endl;
    }else{
        std::cout << "Skipping color range calibration" << std::endl;
    }

    std::cout << "Do you want to calibrate the tracking box (y/n): ";

    std::cin >> response;
    if (response == 'y'){
        calibrateTrackingBox();
        cv::Rect t = parseTrackingBox(TRACKING_BOX_FILE);
        std::cout << "Saved tracking box: \n" << t << std::endl;
        

    }else{
        std::cout << "Skipping tracking box calibration" << std::endl;
    }

    return 0;
}