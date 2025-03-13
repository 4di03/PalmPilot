#ifndef FAST_TRACKER_H
#define FAST_TRACKER_H

#include <opencv2/opencv.hpp>
#include "handTracking.h"
#include "handTracker.h"
#include "removeFace.h"
#include "backgroundSubtraction.h"
#include "calibration.h"
#include "constants.h"
#include "math/kCurvature.h"
#include "math/convexityDefects.h"
#include "math/maxInscribingCircle.h"



// Strategy Classes
class ContourFilterStrategy
{
public:
    virtual std::vector<cv::Point> filterContour(std::vector<std::vector<cv::Point>> contours) = 0;
    virtual ~ContourFilterStrategy() = default;  // ✅ Virtual destructor
};

class ValidContourStrategy
{
public:
    virtual std::vector<std::vector<cv::Point>> getValidContours(std::vector<std::vector<cv::Point>> contours) = 0;
    virtual ~ValidContourStrategy() = default;  // ✅ Virtual destructor
};

class MaxAreaFilter : public ContourFilterStrategy
{
public:
    std::vector<cv::Point> filterContour(std::vector<std::vector<cv::Point>> contours) override;
};

class SolidityFilter : public ValidContourStrategy
{
public:
    std::vector<std::vector<cv::Point>> getValidContours(std::vector<std::vector<cv::Point>> contours) override;
};

class AreaFilter : public ValidContourStrategy
{
public:
    std::vector<std::vector<cv::Point>> getValidContours(std::vector<std::vector<cv::Point>> contours) override;
};

class CircularityFilter : public ValidContourStrategy
{
public:
    std::vector<std::vector<cv::Point>> getValidContours(std::vector<std::vector<cv::Point>> contours) override;
};

class CompositeFilter : public ContourFilterStrategy
{
public:
    std::vector<ValidContourStrategy *> validFilters;
    ContourFilterStrategy *finalSelector;

    CompositeFilter(std::vector<ValidContourStrategy *> validFilters, ContourFilterStrategy *finalSelector)
    {
        this->validFilters = validFilters;
        this->finalSelector = finalSelector;
    }
    std::vector<cv::Point> filterContour(std::vector<std::vector<cv::Point>> contours) override;
};

// Post-Processing Strategies
class HandMaskPostProcessingStrategy
{
public:
    virtual cv::Mat postProcess(cv::Mat &mask) = 0;
    virtual ~HandMaskPostProcessingStrategy() = default;  // ✅ Virtual destructor

};

class GaussianBlurPostProcessing : public HandMaskPostProcessingStrategy
{
private:
    int blurSize;

public:
    GaussianBlurPostProcessing(int blurSize = 5);
    cv::Mat postProcess(cv::Mat &mask) override;
};

class DilationPostProcessing : public HandMaskPostProcessingStrategy
{
private:
    int dilationIterations;
    cv::Mat kernel;

public:
    DilationPostProcessing(int dilationIterations = 4, int kernelSize = 5){
        this->dilationIterations = dilationIterations;
        this->kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));

    }

    cv::Mat postProcess(cv::Mat &mask) override;
};

class ClosingPostProcessing : public HandMaskPostProcessingStrategy
{
private:
    int closingIterations;
    cv::Mat kernel;

public:
    ClosingPostProcessing(int closingIterations = 4, int kernelSize = 5)
    {
        this->closingIterations = closingIterations;
        this->kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
    }

    cv::Mat postProcess(cv::Mat &mask) override;
};

class OpeningPostProcessing : public HandMaskPostProcessingStrategy
{
private:
    int openingIterations;

public:
    OpeningPostProcessing(int openingIterations = 4);
    cv::Mat postProcess(cv::Mat &mask) override;
};

class CompositePostProcessing : public HandMaskPostProcessingStrategy
{
private:
    std::vector<HandMaskPostProcessingStrategy *> postProcessingStrategies;

public:
    CompositePostProcessing(std::vector<HandMaskPostProcessingStrategy *> postProcessingStrategies);
    cv::Mat postProcess(cv::Mat &mask) override;
};

class ErosionPostProcessing : public HandMaskPostProcessingStrategy
{
private:
    int erosionIterations;

public:
    ErosionPostProcessing(int erosionIterations = 4);
    cv::Mat postProcess(cv::Mat &mask) override;
};

class IdentityPostProcessing : public HandMaskPostProcessingStrategy
{
public:
    cv::Mat postProcess(cv::Mat &mask) override;
};

// Hand Mask Strategy
class HandMaskStrategy
{
private:
    HandMaskPostProcessingStrategy *postProcessingStrategy;

public:
    HandMaskStrategy(HandMaskPostProcessingStrategy *postProcessingStrategy);
    cv::Mat makeHandMask(const cv::Mat &image);
    ~HandMaskStrategy(){
        delete postProcessingStrategy;
    }
};

// FastTracker Class
class FastTracker : public HandTracker
{
private:
    ContourFilterStrategy *filterStrategy;
    HandMaskStrategy *maskStrategy;

public:
    FastTracker(ContourFilterStrategy *filterStrategy, HandMaskStrategy *maskStrategy);
    HandData getHandData(const cv::Mat &image) override;
    ~FastTracker(){
        delete filterStrategy;
        delete maskStrategy;
    }
};

FastTracker* initBestTracker();
#endif // FAST_TRACKER_H
