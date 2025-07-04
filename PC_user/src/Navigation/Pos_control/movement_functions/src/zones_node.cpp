#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void publishStaticTF(double x, double y, const std::string& child_frame, tf2_ros::StaticTransformBroadcaster& broadcaster) {
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "Log_origin";
    tf.child_frame_id = child_frame;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    broadcaster.sendTransform(tf);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "zone_tf_spawner");
    ros::NodeHandle nh;

    tf2_ros::StaticTransformBroadcaster broadcaster;

    // Magenta zones (M_Z<fila><col>)
    for (int row = 1; row <= 7; ++row) {
        for (int col = 1; col <= 8; ++col) {
            double x = -row + 0.5;
            double y = col - 0.5;
            std::string frame_id = "M_Z" + std::to_string(row) + std::to_string(col);
            publishStaticTF(x, y, frame_id, broadcaster);
        }
    }

    // Cyan zones (C_Z<fila><col>)
    for (int row = 1; row <= 7; ++row) {
        for (int col = 1; col <= 8; ++col) {
            double x = row - 0.5;
            double y = col - 0.5;
            std::string frame_id = "C_Z" + std::to_string(row) + std::to_string(col);
            publishStaticTF(x, y, frame_id, broadcaster);
        }
    }

    ros::spin();
    return 0;
}
