#include <ros/ros.h>
#include <vision_logistics/camera_processor.hpp>
#include <vision_logistics/RunTask.h>

CameraProcessor processor;

bool runTaskCallback(vision_logistics::RunTask::Request &req,
                     vision_logistics::RunTask::Response &res) {
    if (!processor.hasImage()) {
        ROS_WARN("No image received yet.");
        res.success = false;
        return true;
    }

    if (req.task_name == "find_aluminum") {
        res.success = processor.findAluminumPlatform(res.error_x, res.error_y);
    } else if (req.task_name == "find_conveyor") {
        int err_x;
        res.success = processor.findConveyor(err_x);
        res.error_x = err_x;
        res.error_y = 0;
    } else if (req.task_name == "center_conveyor") {
        int err_x;
        res.success = processor.centerConveyor(err_x);
        res.error_x = err_x;
        res.error_y = 0;
    } else if (req.task_name == "find_piece") {
        int err_y;
        res.success = processor.findPieceOnConveyor(err_y);
        res.error_x = 0;
        res.error_y = err_y;
    } else if (req.task_name == "find_end") {
        int err_y;
        res.success = processor.findEndOfConveyor(err_y);
        res.error_x = 0;
        res.error_y = err_y;
    } else {
        ROS_WARN("Unknown task: %s", req.task_name.c_str());
        res.success = false;
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_tasks_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/realsense/color/image_raw", 1, &CameraProcessor::imageCallback, &processor);
    ros::ServiceServer srv = nh.advertiseService("/vision/run_camera_task", runTaskCallback);

    ros::spin();
    return 0;
}
