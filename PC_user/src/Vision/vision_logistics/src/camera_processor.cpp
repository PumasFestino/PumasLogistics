#include <vision_logistics/camera_processor.hpp>

CameraProcessor::CameraProcessor() : image_received_(false) {}

void CameraProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        latest_image_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        image_received_ = true;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

bool CameraProcessor::hasImage() const {
    return image_received_;
}

// 🎯 Encuentra plataforma de aluminio por color claro
bool CameraProcessor::findAluminumPlatform(int& err_x, int& err_y) {
    cv::Mat gray, mask;
    cv::cvtColor(latest_image_, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, 200, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return false;

    auto max_contour = *std::max_element(contours.begin(), contours.end(),
        [](auto& a, auto& b) { return cv::contourArea(a) < cv::contourArea(b); });

    cv::Rect bbox = cv::boundingRect(max_contour);

    err_x = bbox.x;
    err_y = bbox.y + bbox.height;

    return true;
}

// 🎯 Encuentra banda transportadora negra (mancha más grande)
bool CameraProcessor::findConveyor(int& err_x) {
    cv::Mat gray, mask;
    cv::cvtColor(latest_image_, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, 50, 255, cv::THRESH_BINARY_INV);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;

    auto max_contour = *std::max_element(contours.begin(), contours.end(),
        [](auto& a, auto& b) { return cv::contourArea(a) < cv::contourArea(b); });

    cv::Rect bbox = cv::boundingRect(max_contour);
    err_x = bbox.x + bbox.width / 2 - latest_image_.cols / 2;

    return true;
}

// 🎯 Ajuste fino de centrado usando Canny
bool CameraProcessor::centerConveyor(int& err_x) {
    cv::Mat gray, mask, edges;
    cv::cvtColor(latest_image_, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, 50, 255, cv::THRESH_BINARY_INV);
    cv::Canny(mask, edges, 100, 200);

    std::vector<cv::Point> edge_points;
    cv::findNonZero(edges, edge_points);
    if (edge_points.empty()) return false;

    cv::Moments m = cv::moments(edge_points);
    int cx = int(m.m10 / m.m00);
    err_x = cx - latest_image_.cols / 2;

    return true;
}

// 🎯 Encuentra objeto en banda (algo no negro)
bool CameraProcessor::findPieceOnConveyor(int& err_y) {
    cv::Mat hsv, mask;
    cv::cvtColor(latest_image_, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 60), cv::Scalar(180, 255, 255), mask);  // todo menos negro

    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);
    if (points.empty()) return false;

    cv::Moments m = cv::moments(points);
    int cy = int(m.m01 / m.m00);
    err_y = cy - latest_image_.rows / 2;

    return true;
}

// 🎯 Encuentra final de la banda (línea blanca después del negro)
bool CameraProcessor::findEndOfConveyor(int& err_y) {
    cv::Mat gray;
    cv::cvtColor(latest_image_, gray, cv::COLOR_BGR2GRAY);

    for (int y = gray.rows - 1; y >= 0; --y) {
        cv::Scalar row_mean = cv::mean(gray.row(y));
        if (row_mean[0] > 80) { // asume fin de banda
            err_y = y - gray.rows / 2;
            return true;
        }
    }

    return false;
}
