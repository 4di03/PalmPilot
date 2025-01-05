#include "calibration.h"

int main(){
    calibrateColorRange();

    colorRange c = parseColorRange("data/color_range.yaml");
    std::cout << "Saved color range: \n" << c.toString() << std::endl;
    
    setBackground();

    
    return 0;
}