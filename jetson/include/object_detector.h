#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>

struct Detection {
    int classId;
    float confidence;
    cv::Rect box;
    std::string label;
};

class ObjectDetector {
public:
    ObjectDetector();
    bool loadModel(const std::string& modelPath, const std::string& configPath);
    std::vector<Detection> detect(const cv::Mat& frame);
    void setConfidenceThreshold(float thresh) { confThreshold = thresh; }
    void setNMSThreshold(float thresh) { nmsThreshold = thresh; }
    
private:
    cv::dnn::Net net;
    std::vector<std::string> classNames;
    float confThreshold = 0.5f;
    float nmsThreshold = 0.4f;
    int inputWidth = 416;
    int inputHeight = 416;
    
    std::vector<cv::String> getOutputNames();
    void postProcess(const cv::Mat& frame, const std::vector<cv::Mat>& outs,
                     std::vector<Detection>& detections);
};
