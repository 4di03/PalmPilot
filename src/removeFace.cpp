#include "removeFace.h"
#include <opencv2/opencv.hpp>
#define FACE_CLASSFIER_FILE "data/models/haarcascade_frontalface_alt.xml"
class FaceRemover{
    public:
        FaceRemover(){
            if (!faceCascade.load(FACE_CLASSFIER_FILE))
            throw std::runtime_error("can't load file for faceCascade");
        }
        // returns a new image with faces blacked out
        cv::Mat removeFace(const cv::Mat inputImg){
            std::vector<cv::Rect> faces;
            cv::Mat frameGray;
            cv::Mat output = inputImg.clone();

            cvtColor(inputImg, frameGray, cv::COLOR_BGR2GRAY);
            equalizeHist(frameGray, frameGray);

            faceCascade.detectMultiScale(frameGray, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(120, 120));

            for (size_t i = 0; i < faces.size(); i++) {
                cv::rectangle( 
                    output,
                    cv::Point(faces[i].x, faces[i].y),
                    cv::Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height),
                    cv::Scalar(0, 0, 0),
                    -1
                );
            }
            return output;

        }
    private:
        cv::CascadeClassifier faceCascade;
};
// removes the face from the input image
cv::Mat removeFace(const cv::Mat inputImg){
    // only loads in the model the ifrst time this function is called
    static FaceRemover faceRemover = FaceRemover();
    cv::Mat output = faceRemover.removeFace(inputImg);
    cv::imshow("With Face", inputImg);
    cv::imshow("Face Removed", output);
    return output;
}
