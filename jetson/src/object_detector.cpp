#include "object_detector.h"

ObjectDetector::ObjectDetector() {}

bool ObjectDetector::loadModel(const std::string& modelPath, const std::string& configPath) {
    try {
        net = cv::dnn::readNetFromDarknet(configPath, modelPath);
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        return true;
    } catch (...) {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        return true;
    }
}

std::vector<cv::String> ObjectDetector::getOutputNames() {
    std::vector<cv::String> names;
    auto outLayers = net.getUnconnectedOutLayers();
    auto layerNames = net.getLayerNames();
    names.resize(outLayers.size());
    for (size_t i = 0; i < outLayers.size(); i++) {
        names[i] = layerNames[outLayers[i] - 1];
    }
    return names;
}

std::vector<Detection> ObjectDetector::detect(const cv::Mat& frame) {
    std::vector<Detection> detections;
    
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1/255.0, 
                           cv::Size(inputWidth, inputHeight),
                           cv::Scalar(0,0,0), true, false);
    net.setInput(blob);
    
    std::vector<cv::Mat> outs;
    net.forward(outs, getOutputNames());
    
    postProcess(frame, outs, detections);
    return detections;
}

void ObjectDetector::postProcess(const cv::Mat& frame, 
                                  const std::vector<cv::Mat>& outs,
                                  std::vector<Detection>& detections) {
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    
    for (const auto& out : outs) {
        for (int i = 0; i < out.rows; i++) {
            const float* data = out.ptr<float>(i);
            float confidence = data[4];
            
            if (confidence > confThreshold) {
                cv::Mat scores = out.row(i).colRange(5, out.cols);
                cv::Point classIdPoint;
                double maxScore;
                cv::minMaxLoc(scores, 0, &maxScore, 0, &classIdPoint);
                
                if (maxScore > confThreshold) {
                    int cx = (int)(data[0] * frame.cols);
                    int cy = (int)(data[1] * frame.rows);
                    int w = (int)(data[2] * frame.cols);
                    int h = (int)(data[3] * frame.rows);
                    int x = cx - w/2;
                    int y = cy - h/2;
                    
                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)maxScore);
                    boxes.push_back(cv::Rect(x, y, w, h));
                }
            }
        }
    }
    
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    
    for (int idx : indices) {
        Detection det;
        det.classId = classIds[idx];
        det.confidence = confidences[idx];
        det.box = boxes[idx];
        detections.push_back(det);
    }
}
