#ifndef CAMERA_PROCESSOR_HPP
#define CAMERA_PROCESSOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class CameraProcessor {
public:
    CameraProcessor();

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    bool hasImage() const;

    bool findAluminumPlatform(int& err_x, int& err_y);
    bool findConveyor(int& err_x);
    bool centerConveyor(int& err_x);
    bool findPieceOnConveyor(int& err_y);
    bool findEndOfConveyor(int& err_y);

private:
    cv::Mat latest_image_;
    bool image_received_;
};

#endif
