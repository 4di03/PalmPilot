#include "removeFace.h"
#include <opencv2/opencv.hpp>
#define FRONTAL_FACE_MODEL "data/models/haarcascade_frontalface_alt.xml"
#define PROFILE_FACE_MODEL "data/models/haarcascade_profileface.xml"
// TODO: fix facial remover, maybe try NN approach
class FaceRemover {
public:
    FaceRemover() {
        // Load frontal face model
        if (!frontalFaceCascade.load(FRONTAL_FACE_MODEL)) {
            throw std::runtime_error("Can't load frontal face model");
        }

        // Load profile face model
        if (!profileFaceCascade.load(PROFILE_FACE_MODEL)) {
            throw std::runtime_error("Can't load profile face model");
        }
    }

    // Returns a new image with faces blacked out
    cv::Mat removeFace(const cv::Mat& inputImg) {
        std::vector<cv::Rect> faces;
        std::vector<cv::Rect> profiles;
        cv::Mat frameGray;
        cv::Mat output = inputImg.clone();

        // Convert to grayscale and equalize histogram
        cv::cvtColor(inputImg, frameGray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(frameGray, frameGray);

        // Detect frontal faces
        frontalFaceCascade.detectMultiScale(frameGray, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE);

        // Detect profile faces
        profileFaceCascade.detectMultiScale(frameGray, profiles, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE);

        // Blackout frontal faces
        for (const auto& face : faces) {
            cv::rectangle(output, face, cv::Scalar(0, 0, 0), -1);
        }

        // Blackout profile faces
        for (const auto& profile : profiles) {
            cv::rectangle(output, profile, cv::Scalar(0, 0, 0), -1);
        }

        return output;
    }

private:
    cv::CascadeClassifier frontalFaceCascade;
    cv::CascadeClassifier profileFaceCascade;
};

// Removes the face from the input image
cv::Mat removeFace(const cv::Mat& inputImg) {
    // Only loads in the models the first time this function is called
    static FaceRemover faceRemover;
    cv::Mat output = faceRemover.removeFace(inputImg);
    cv::imshow("With Face", inputImg);
    cv::imshow("Face Removed", output);
    return output;
}