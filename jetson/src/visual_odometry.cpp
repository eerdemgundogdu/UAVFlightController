#include "visual_odometry.h"

VisualOdometry::VisualOdometry() {
    currentPose.rotation = cv::Mat::eye(3, 3, CV_64F);
    currentPose.translation = cv::Mat::zeros(3, 1, CV_64F);
    trajectory = cv::Mat::zeros(600, 600, CV_8UC3);
}

void VisualOdometry::init(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    K = cameraMatrix.clone();
    D = distCoeffs.clone();
    initialized = false;
}

void VisualOdometry::reset() {
    currentPose.rotation = cv::Mat::eye(3, 3, CV_64F);
    currentPose.translation = cv::Mat::zeros(3, 1, CV_64F);
    initialized = false;
    prevPoints.clear();
}

void VisualOdometry::detectFeatures(const cv::Mat& frame, 
                                     std::vector<cv::Point2f>& points) {
    cv::goodFeaturesToTrack(frame, points, 2000, 0.01, 10);
}

void VisualOdometry::trackFeatures(const cv::Mat& frame,
                                    std::vector<cv::Point2f>& prevPts,
                                    std::vector<cv::Point2f>& currPts,
                                    std::vector<uchar>& status) {
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prevFrame, frame, prevPts, currPts, status, err);
}

void VisualOdometry::estimateMotion(const std::vector<cv::Point2f>& prevPts,
                                     const std::vector<cv::Point2f>& currPts) {
    cv::Mat E = cv::findEssentialMat(currPts, prevPts, K, cv::RANSAC, 0.999, 1.0);
    cv::Mat R, t;
    cv::recoverPose(E, currPts, prevPts, K, R, t);
    
    currentPose.translation = currentPose.translation + 
                              currentPose.rotation * t;
    currentPose.rotation = R * currentPose.rotation;
}

bool VisualOdometry::processFrame(const cv::Mat& frame) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    if (!initialized) {
        detectFeatures(gray, prevPoints);
        gray.copyTo(prevFrame);
        initialized = true;
        return false;
    }
    
    std::vector<cv::Point2f> currPoints;
    std::vector<uchar> status;
    trackFeatures(gray, prevPoints, currPoints, status);
    
    std::vector<cv::Point2f> goodPrev, goodCurr;
    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            goodPrev.push_back(prevPoints[i]);
            goodCurr.push_back(currPoints[i]);
        }
    }
    
    if (goodPrev.size() >= 8) {
        estimateMotion(goodPrev, goodCurr);
    }
    
    if (goodCurr.size() < 500) {
        detectFeatures(gray, prevPoints);
    } else {
        prevPoints = goodCurr;
    }
    
    gray.copyTo(prevFrame);
    currentPose.timestamp = cv::getTickCount() / cv::getTickFrequency();
    
    int x = int(currentPose.translation.at<double>(0)) + 300;
    int y = int(currentPose.translation.at<double>(2)) + 300;
    if (x >= 0 && x < 600 && y >= 0 && y < 600) {
        cv::circle(trajectory, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), 2);
    }
    
    return true;
}
