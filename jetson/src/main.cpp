#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include "object_detector.h"
#include "visual_odometry.h"
#include "serial_comm.h"

std::atomic<bool> running(true);
SerialPort serial;

struct TargetInfo {
    bool detected = false;
    float centerX = 0, centerY = 0;
    float area = 0;
};

TargetInfo currentTarget;
std::mutex targetMutex;

void visionThread() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera" << std::endl;
        return;
    }
    
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    
    ObjectDetector detector;
    detector.loadModel("yolov4-tiny.weights", "yolov4-tiny.cfg");
    
    cv::Mat K = (cv::Mat_<double>(3,3) << 
        500, 0, 320,
        0, 500, 240,
        0, 0, 1);
    cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
    
    VisualOdometry vo;
    vo.init(K, D);
    
    cv::Mat frame;
    while (running) {
        cap >> frame;
        if (frame.empty()) continue;
        
        auto detections = detector.detect(frame);
        
        {
            std::lock_guard<std::mutex> lock(targetMutex);
            currentTarget.detected = !detections.empty();
            if (!detections.empty()) {
                auto& det = detections[0];
                currentTarget.centerX = det.box.x + det.box.width / 2.0f;
                currentTarget.centerY = det.box.y + det.box.height / 2.0f;
                currentTarget.area = det.box.width * det.box.height;
            }
        }
        
        vo.processFrame(frame);
        
        for (const auto& det : detections) {
            cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
        }
        
        cv::imshow("UAV Vision", frame);
        cv::imshow("Trajectory", vo.getTrajectory());
        
        if (cv::waitKey(1) == 27) running = false;
    }
}

void controlThread() {
    const float frameWidth = 640, frameHeight = 480;
    const float deadzone = 20;
    
    while (running) {
        TargetInfo target;
        {
            std::lock_guard<std::mutex> lock(targetMutex);
            target = currentTarget;
        }
        
        float rollCmd = 0, pitchCmd = 0;
        
        if (target.detected) {
            float errorX = target.centerX - frameWidth / 2;
            float errorY = target.centerY - frameHeight / 2;
            
            if (std::abs(errorX) > deadzone) {
                rollCmd = errorX * 0.02f;
            }
            if (std::abs(errorY) > deadzone) {
                pitchCmd = -errorY * 0.02f;
            }
            
            rollCmd = std::max(-15.0f, std::min(rollCmd, 15.0f));
            pitchCmd = std::max(-15.0f, std::min(pitchCmd, 15.0f));
        }
        
        serial.sendCommand(rollCmd, pitchCmd, 0);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

int main(int argc, char** argv) {
    std::cout << "UAV Vision System Starting..." << std::endl;
    
    std::string port = "/dev/ttyACM0";
    if (argc > 1) port = argv[1];
    
    if (!serial.open(port, 115200)) {
        std::cerr << "Failed to open serial port: " << port << std::endl;
    } else {
        std::cout << "Serial connected: " << port << std::endl;
    }
    
    std::thread vision(visionThread);
    std::thread control(controlThread);
    
    vision.join();
    control.join();
    
    return 0;
}
