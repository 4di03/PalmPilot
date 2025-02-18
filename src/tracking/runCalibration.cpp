#include "calibration.h"

int main(){
    calibrateColorRange();

    colorRange c = parseColorRange(COLOR_RANGE_FILE);
    std::cout << "Saved color range: \n" << c.toString() << std::endl;
    
    setBackground();

    
    return 0;
}