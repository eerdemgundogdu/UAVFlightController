#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

struct Pose {
    cv::Mat rotation;
    cv::Mat translation;
    double timestamp;
};

class VisualOdometry {
public:
    VisualOdometry();
    void init(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
    bool processFrame(const cv::Mat& frame);
    Pose getCurrentPose() const { return currentPose; }
    cv::Mat getTrajectory() const { return trajectory; }
    void reset();
    
private:
    cv::Mat K;
    cv::Mat D;
    cv::Mat prevFrame;
    std::vector<cv::Point2f> prevPoints;
    Pose currentPose;
    cv::Mat trajectory;
    bool initialized = false;
    
    void detectFeatures(const cv::Mat& frame, std::vector<cv::Point2f>& points);
    void trackFeatures(const cv::Mat& frame, std::vector<cv::Point2f>& prevPts,
                       std::vector<cv::Point2f>& currPts, std::vector<uchar>& status);
    void estimateMotion(const std::vector<cv::Point2f>& prevPts,
                        const std::vector<cv::Point2f>& currPts);
};
