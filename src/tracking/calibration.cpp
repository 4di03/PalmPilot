#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>

#include "cam.h"
#include "calibration.h"
#include "backgroundSubtraction.h"
#define TL_X 700    
#define TL_Y 300
#define SIZE 20
#define COLOR_CONVERSION cv::COLOR_BGR2YCrCb


int calibration_rect_tl_x = TL_X;
int calibration_rect_tl_y = TL_Y;
// Callback function for mouse events
void onMouse(int event, int x, int y, int, void*) {
    //std::cout << "Mouse event: " << event << " at (" << x << ", " << y << ")" << std::endl;
    if (event == cv::EVENT_LBUTTONDOWN) { // Left mouse button click
        calibration_rect_tl_x = x - SIZE / 2; // Center the rectangle at the click
        calibration_rect_tl_y = y - SIZE / 2;
    }
}
// Callback function for trackbar updates (does nothing but is required)
void on_trackbar(int, void*) {}


// Splits a string based on a delimiter
std::vector<std::string> split(const std::string& str, const std::string& delimiter) {
    std::vector<std::string> tokens;
    size_t start = 0;
    size_t end = str.find(delimiter);

    while (end != std::string::npos) {
        tokens.push_back(str.substr(start, end - start));
        start = end + delimiter.length();
        end = str.find(delimiter, start);
    }

    tokens.push_back(str.substr(start));
    return tokens;
}



// Parses the color range from a file
colorRange parseColorRange( std::string path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file to read color range." << std::endl;
        exit(1);
    }

    // Read the lines
    std::string firstLine, secondLine;
    std::getline(file, firstLine);
    std::getline(file, secondLine);

    // Parse "Lower: " line
    std::vector<std::string> lower = split(firstLine, "Lower: ");
    if (lower.size() != 2) {
        std::cerr << "Error: Could not parse lower color range." << std::endl;
        std::cerr << "Got: " << firstLine << std::endl;
        exit(1);
    }

    // Parse "Upper: " line
    std::vector<std::string> upper = split(secondLine, "Upper: ");
    if (upper.size() != 2) {
        std::cerr << "Error: Could not parse upper color range." << std::endl;
        std::cerr << "Got: " << secondLine << std::endl;
        exit(1);
    }

    // Extract lower color range values
    std::vector<int> lowerValues;
    std::stringstream lowerStream(lower[1]);
    std::string token;
    while (std::getline(lowerStream, token, ',')) {
        lowerValues.push_back(std::stoi(token));
    }
    if (lowerValues.size() != 3) {
        std::cerr << "Error: Lower color range must have exactly 3 values." << std::endl;
        exit(1);
    }

    // Extract upper color range values
    std::vector<int> upperValues;
    std::stringstream upperStream(upper[1]);
    while (std::getline(upperStream, token, ',')) {
        upperValues.push_back(std::stoi(token));
    }
    if (upperValues.size() != 3) {
        std::cerr << "Error: Upper color range must have exactly 3 values." << std::endl;
        exit(1);
    }

    // Construct the colorRange object
    colorRange range;
    range.lower = cv::Scalar(lowerValues[0], lowerValues[1], lowerValues[2]);
    range.upper = cv::Scalar(upperValues[0], upperValues[1], upperValues[2]);

    return range;
}



// Allows user to set optinmal color range for hand segmentation
void calibrateColorRange() {
    // Load the background image
    VideoStream stream(0);

    // Create a window
    const std::string windowName = "Calibration";
    const std::string originalWindow = "Original";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(originalWindow, cv::WINDOW_AUTOSIZE);

    // Set mouse callback for the calibration window
    cv::setMouseCallback(originalWindow, onMouse);



    int c1Min = 0, c1Max = 255;
    int c2Min = 0, c2Max = 255;
    int c3Min = 0, c3Max = 255;

    // check if color range file exists
    std::ifstream colorRangeFile(COLOR_RANGE_FILE);
    if (colorRangeFile.is_open()) {
        std::cout << "Color range file found. Loading previous color range." << std::endl;
        colorRangeFile.close();
        colorRange previousRange = parseColorRange(COLOR_RANGE_FILE);

        std::cout << "Setting previous color range: " << std::endl;

        // Variables for YCrCb range
        c1Min = previousRange.lower[0];
        c1Max = previousRange.upper[0];
        c2Min = previousRange.lower[1];
        c2Max = previousRange.upper[1];
        c3Min = previousRange.lower[2];
        c3Max = previousRange.upper[2];

    } 


    // update variables to range

    // Create trackbars
    cv::createTrackbar("C1 Min", windowName, &c1Min, 255, on_trackbar);
    cv::createTrackbar("C1 Max", windowName, &c1Max, 255, on_trackbar);
    cv::createTrackbar("C2 Min", windowName, &c2Min, 255, on_trackbar);
    cv::createTrackbar("C2 Max", windowName, &c2Max, 255, on_trackbar);
    cv::createTrackbar("C3 Min", windowName, &c3Min, 255, on_trackbar);
    cv::createTrackbar("C3 Max", windowName, &c3Max, 255, on_trackbar);

    std::cout << "Please slide the trackbars to set the optimal color range for hand segmentation. Don't worry if elements in the background also appear in the segmenetaiton, those will be removed by the background subtraction. Make sure the hand is as white as possible." << std::endl;
    std::cout << "You are calibrating in the " << COLOR_CONVERSION << " color space." << std::endl;
    std::cout << "Press 'q' or 'ESC' to exit the calibration and save the results.." << std::endl;
    // Calibration loop
    while (true) {

        // Convert the background image to YCrCb
        cv::Mat image;
        cv::cvtColor(stream.getFrame(), image, COLOR_CONVERSION);

        // get average YCrCb values ƒof the box
        cv::Mat roi = image(cv::Rect(calibration_rect_tl_x, calibration_rect_tl_y, SIZE, SIZE));
        cv::Scalar avg = cv::mean(roi);
        cv::putText(image, "Avg Y: " + std::to_string(avg[0]), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        cv::putText(image, "Avg Cr: " + std::to_string(avg[1]), cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        cv::putText(image, "Avg Cb: " + std::to_string(avg[2]), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

        // draw a box and the average YCrCb values of it to help user calibrate
        cv::rectangle(image, cv::Point(calibration_rect_tl_x, calibration_rect_tl_y), cv::Point(calibration_rect_tl_x + SIZE, calibration_rect_tl_y + SIZE), cv::Scalar(0, 255, 0), 2);

        // Show the mask
        cv::Mat mask;
        cv::inRange(image, cv::Scalar(c1Min, c2Min, c3Min), cv::Scalar(c1Max, c2Max, c3Max), mask);

        cv::imshow(windowName, mask);
        cv::imshow(originalWindow, image);

        // Exit if the user presses 'q'
        char key = static_cast<char>(cv::waitKey(30)); // updates to slider variables are made here
        if (key == 'q' || key == 27) break; // 'q' or 'ESC' to quit
    }

    cv::destroyAllWindows();


    cv::Scalar lower(c1Min, c2Min, c3Min);
    cv::Scalar upper(c1Max, c2Max, c3Max);
    colorRange range{lower, upper};
    // Save the color range to a file
    std::ofstream file;
    file.open(COLOR_RANGE_FILE);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file to save color range." << std::endl;
        return;
    }
    // Write the color range to the file as 
    file << range.toString() <<std::endl;
    file.close();

}
// prompt the user to take a snapshot of their background to save it for background subtraction
void setBackground(){
    VideoStream stream(0);
    std::cout << "Please exit the frame and press 'q' to save a snapshot of the background. Please do not shift the camera in future use after saving this background." << std::endl;
    while(true){
        cv::Mat frame = stream.getFrame();
        cv::imshow("Background", frame);
        char key = static_cast<char>(cv::waitKey(30)); // updates to slider variables are made here
        if (key == 'q'){
            cv::imwrite(BACKGROUND_FILE_LOC, frame);
            break;
        }; // 'q' or 'ESC' to quit
    }

}

// Parses the tracking box from a file
cv::Rect parseTrackingBox(std::string path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file to read tracking box." << std::endl;
        exit(1);
    }

    // Read the line
    std::string line;
    std::getline(file, line);

    std::vector<std::string> values = split(line, "top_left_x: ");
    if (values.size() != 2) {
        std::cerr << "Error: Could not parse top left x value." << std::endl;
        std::cerr << "Got: " << line << std::endl;
        exit(1);
    }
    int tl_x = std::stoi(values[1]);

    std::getline(file, line);
    values = split(line, "top_left_y: ");
    if (values.size() != 2) {
        std::cerr << "Error: Could not parse top left y value." << std::endl;
        std::cerr << "Got: " << line << std::endl;
        exit(1);
    }


    int tl_y = std::stoi(values[1]);

    std::getline(file, line);
    values = split(line, "width: ");
    if (values.size() != 2) {
        std::cerr << "Error: Could not parse width value." << std::endl;
        std::cerr << "Got: " << line << std::endl;
        exit(1);
    }

    int width = std::stoi(values[1]);

    std::getline(file, line);

    values = split(line, "height: ");
    if (values.size() != 2) {
        std::cerr << "Error: Could not parse height value." << std::endl;
        std::cerr << "Got: " << line << std::endl;
        exit(1);
    }


    int height = std::stoi(values[1]);


    return cv::Rect(tl_x, tl_y, width, height);

}


/**
 * Callback function for dragging and resizing a rectangle
 */
void dragRectCallback(int event, int x, int y, int flags, void* userdata) {
    auto* rectPtr = static_cast<cv::Rect*>(userdata);
    static cv::Point clickOffset;
    static bool dragging = false;
    static int resizeDirection = -1; // 0: Left, 1: Right, 2: Top, 3: Bottom, -1 is not resizing
    constexpr int resizeBorder = 10;

    if (!rectPtr){
        std::cerr << "Error: No rectangle pointer provided." << std::endl;
        exit(1);
    }
    cv::Rect& rect = *rectPtr;

    //std::cout << "Rect: " << rect << std::endl;

    bool onLeft   = (abs(x - rect.x) < resizeBorder);
    bool onRight  = (abs(x - (rect.x + rect.width)) < resizeBorder);
    bool onTop    = (abs(y - rect.y) < resizeBorder);
    bool onBottom = (abs(y - (rect.y + rect.height)) < resizeBorder);
    cv::Point point(x, y);

    switch (event) {
        case cv::EVENT_LBUTTONDOWN:
            if (onLeft) {
                resizeDirection = 0; // Left
            } else if (onRight) {
                resizeDirection = 1; // Right
            } else if (onTop) {
                resizeDirection = 2; // Top

            } else if (onBottom) {
                resizeDirection = 3; // Bottom
            } else if (rect.contains(point)) { 
                std::cout << "Dragging" << std::endl;
                dragging = true;
                resizeDirection = -1;
                clickOffset = point - cv::Point(rect.x, rect.y);
            } else {
                dragging = false;
                resizeDirection = -1;
            }
            break;

        case cv::EVENT_MOUSEMOVE:
            if (dragging) {
                rect.x = x - clickOffset.x;
                rect.y = y - clickOffset.y;
            } else if (resizeDirection != -1) { // Resizing
                switch (resizeDirection) {
                    case 0: rect.width += rect.x - x; rect.x = x; break; // Left
                    case 1: rect.width = x - rect.x; break;             // Right
                    case 2: rect.height += rect.y - y; rect.y = y; break; // Top
                    case 3: rect.height = y - rect.y; break;             // Bottom
                }
                // Ensure minimum size
                rect.width = std::max(rect.width, 20);
                rect.height = std::max(rect.height, 20);
            }
            break;

        case cv::EVENT_LBUTTONUP:
            dragging = false;
            resizeDirection = -1;
            break;
    }
}

/**
 * Drag a rectangle to set the tracking box
 */
void calibrateTrackingBox() {
    // Load the background image
    VideoStream stream(0);
    // Create a window
    const std::string windowName = "Calibration";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

    // Load previous tracking box if available
    std::ifstream trackingBoxFile(TRACKING_BOX_FILE);
    cv::Rect previousBox;
    if (trackingBoxFile.is_open()) {
        std::cout << "Tracking box file found. Loading previous tracking box." << std::endl;
        trackingBoxFile.close();
        previousBox = parseTrackingBox(TRACKING_BOX_FILE);
    } else {
        std::cout << "No tracking box file found. Setting default tracking box." << std::endl;
        previousBox = cv::Rect(100, 100, 100, 100);
    }

    // Set mouse callback
    cv::setMouseCallback(windowName, dragRectCallback, &previousBox);

    std::cout << "Please drag the rectangle to set the tracking box. Make sure the box is not obstructed by the face and is comfortable for the user." << std::endl;
    std::cout << "Press 'q' or 'ESC' to exit the calibration and save the results.." << std::endl;
    // Calibration loop
    while (true) {

        // Convert the background image to YCrCb
        cv::Mat image = stream.getFrame();

        // draw a box and the average YCrCb values of it to help user calibrate

        cv::rectangle(image, previousBox, cv::Scalar(0, 255, 0), 2);

        cv::imshow(windowName, image);

        // Exit if the user presses 'q'
        char key = static_cast<char>(cv::waitKey(1)); // updates to slider variables are made here
        if (key == 'q' || key == 27) break; // 'q' or 'ESC' to quit
    }


    // Save the tracking box to a file


    std::ofstream file;
    file.open(TRACKING_BOX_FILE);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file to save tracking box." << std::endl;
        return;
    }
    // Write the tracking box to the file as
    file << "top_left_x: " << previousBox.x << std::endl;
    file << "top_left_y: " << previousBox.y << std::endl;
    file << "width: " << previousBox.width << std::endl;
    file << "height: " << previousBox.height << std::endl;
    file.close();

    cv::destroyAllWindows();

}