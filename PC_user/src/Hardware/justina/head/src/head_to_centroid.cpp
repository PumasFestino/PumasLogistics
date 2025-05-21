#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>

class HeadTracker
{
public:
    HeadTracker()
        : tracking_active_(false), current_pan_(0.0), current_tilt_(0.0),
          filtered_pan_(0.0), filtered_tilt_(0.0),
          timeout_s_(0.3), persist_enabled_(true)
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        service_ = nh.advertiseService("/head_tracking/activate", &HeadTracker::activationCallback, this);

        head_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/hardware/head/goal_pose", 1);
        current_pose_sub_ = nh.subscribe("/hardware/head/current_pose", 1, &HeadTracker::currentPoseCallback, this);

        // Parámetros
        pnh.param("max_pan", max_pan_, 1.57);
        pnh.param("max_tilt", max_tilt_, 1.0);
        pnh.param("kp_pan", kp_pan_, 1.0);
        pnh.param("kp_tilt", kp_tilt_, 0.8);
        pnh.param("pan_threshold", pan_threshold_, 0.02);
        pnh.param("tilt_threshold", tilt_threshold_, 0.02);
        pnh.param("alpha_filter", alpha_filter_, 0.8);
        pnh.param("timeout_s", timeout_s_, 0.3);
        pnh.param("persist_enabled", persist_enabled_, true);

        cmd_timer_ = nh.createTimer(ros::Duration(0.05), &HeadTracker::timerCallback, this); // 20Hz

        ROS_INFO("Servicio de seguimiento de cabeza listo");
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    ros::Subscriber centroid_sub_;
    ros::Subscriber current_pose_sub_;
    ros::Publisher head_pub_;
    ros::Timer cmd_timer_;

    bool tracking_active_;
    double current_pan_;
    double current_tilt_;
    double max_pan_;
    double max_tilt_;
    double kp_pan_;
    double kp_tilt_;
    double pan_threshold_;
    double tilt_threshold_;
    double alpha_filter_;
    double timeout_s_;
    bool persist_enabled_;

    double filtered_pan_;
    double filtered_tilt_;
    ros::Time last_centroid_time_;
    std_msgs::Float64MultiArray last_command_;

    bool activationCallback(std_srvs::SetBool::Request &req,
                            std_srvs::SetBool::Response &res)
    {
        tracking_active_ = req.data;

        if (tracking_active_)
        {
            centroid_sub_ = nh_.subscribe("/vision/body_centroid", 1,
                                          &HeadTracker::centroidCallback, this);
            ROS_INFO("Seguimiento activado");
            last_centroid_time_ = ros::Time::now();
        }
        else
        {
            centroid_sub_.shutdown();
            ROS_INFO("Seguimiento desactivado");
        }

        res.success = true;
        res.message = tracking_active_ ? "Tracking activado" : "Tracking desactivado";
        return true;
    }

    void centroidCallback(const geometry_msgs::Point::ConstPtr &msg)
    {
        double x_error = msg->x;
        double y_error = msg->y;
        double z_distance = fmax(msg->z, 0.1);

        double pan_target = kp_pan_ * atan2(-x_error, z_distance);
        double tilt_target = kp_tilt_ * atan2(y_error, z_distance);

        pan_target = std::max(std::min(pan_target, max_pan_), -max_pan_);
        tilt_target = std::max(std::min(tilt_target, max_tilt_), -max_tilt_);

        // Filtro exponencial
        filtered_pan_ = alpha_filter_ * pan_target + (1.0 - alpha_filter_) * filtered_pan_;
        filtered_tilt_ = alpha_filter_ * tilt_target + (1.0 - alpha_filter_) * filtered_tilt_;

        bool pan_valid = fabs(filtered_pan_ - current_pan_) > pan_threshold_;
        bool tilt_valid = fabs(filtered_tilt_ - current_tilt_) > tilt_threshold_;

        if (pan_valid || tilt_valid)
        {
            current_pan_ = filtered_pan_;
            current_tilt_ = filtered_tilt_;

            std_msgs::Float64MultiArray head_cmd;
            head_cmd.data.clear();
            head_cmd.data.push_back(current_pan_);
            head_cmd.data.push_back(current_tilt_);
            last_command_ = head_cmd;
            head_pub_.publish(head_cmd);
        }

        last_centroid_time_ = ros::Time::now();
    }

    void currentPoseCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() >= 2)
        {
            current_pan_ = msg->data[0];
            current_tilt_ = msg->data[1];
            filtered_pan_ = current_pan_;
            filtered_tilt_ = current_tilt_;
        }
    }

    void timerCallback(const ros::TimerEvent &)
    {
        if (!tracking_active_ || !persist_enabled_) return;

        ros::Duration time_since_last = ros::Time::now() - last_centroid_time_;
        if (time_since_last.toSec() > timeout_s_)
        {
            // Repetir último comando (solo si no estamos ya publicando lo mismo)
            if (!last_command_.data.empty())
            {
                head_pub_.publish(last_command_);
                ROS_DEBUG("Persistiendo último comando por pérdida de centroide");
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "head_tracking_node");
    HeadTracker tracker;
    ros::spin();
    return 0;
}
