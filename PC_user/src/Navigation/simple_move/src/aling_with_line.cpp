#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <vector>
#include <cmath>
#include <random>

class LineAligner
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cmd_pub_;
    ros::ServiceServer service_;
    ros::Timer timer_;

    std::vector<std::pair<double, double>> points_;
    geometry_msgs::Twist cmd;
    bool is_align_enabled_;
    bool use_ransac_;

    const int MAX_ITER = 100;
    const double DIST_THRESHOLD = 0.05;

public:
    LineAligner()
    {
        scan_sub_ = nh_.subscribe("/scan", 1, &LineAligner::scanCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        service_ = nh_.advertiseService("/navigation/align_with_line", &LineAligner::toggleAlignment, this);

        is_align_enabled_ = false;
        use_ransac_ = true;
        timer_ = nh_.createTimer(ros::Duration(0.05), &LineAligner::process, this);

        std::cout << "Line Aligner with Hokuyo --- Soft by Joshua M" << std::endl;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        points_.clear();
        for (int i = 0; i < msg->ranges.size(); ++i)
        {
            float range = msg->ranges[i];
            if (range < msg->range_max && range > msg->range_min)
            {
                double angle = msg->angle_min + i * msg->angle_increment;
                double x = range * cos(angle);
                double y = range * sin(angle);
                points_.emplace_back(x, y);
            }
        }
    }

    void process(const ros::TimerEvent&)
    {
        if (!is_align_enabled_ || points_.size() < 2) return;

        double angle_error = 0.0;

        if (use_ransac_)
        {
            angle_error = computeAngleErrorRANSAC(points_);
        }
        else
        {
            angle_error = computeAngleErrorCenterOfMass(points_);
        }

        cmd.angular.z = -0.5 * angle_error; 
        cmd_pub_.publish(cmd);
    }

    double computeAngleErrorRANSAC(const std::vector<std::pair<double, double>>& points)
    {
        int best_inliers = 0;
        double best_angle = 0.0;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, points.size() - 1);

        for (int iter = 0; iter < MAX_ITER; ++iter)
        {
            int idx1 = dis(gen);
            int idx2 = dis(gen);
            if (idx1 == idx2) continue;

            auto p1 = points[idx1];
            auto p2 = points[idx2];

            double dx = p2.first - p1.first;
            double dy = p2.second - p1.second;
            if (dx == 0) continue;

            double angle = atan2(dy, dx);

            int inliers = 0;
            for (const auto& p : points)
            {
                double distance = fabs(dy * p.first - dx * p.second + (p2.first * p1.second - p2.second * p1.first)) / sqrt(dy * dy + dx * dx);
                if (distance < DIST_THRESHOLD) {
                    inliers++;
                }
            }

            if (inliers > best_inliers)
            {
                best_inliers = inliers;
                best_angle = angle;
            }
        }

        best_angle += M_PI_2;

        while (best_angle > M_PI) best_angle -= 2 * M_PI;
        while (best_angle < -M_PI) best_angle += 2 * M_PI;

        return best_angle;
    }

    double computeAngleErrorCenterOfMass(const std::vector<std::pair<double, double>>& points)
    {
        double sum_x = 0.0;
        double sum_y = 0.0;
        int valid_points = points.size();

        for (const auto& p : points) {
            sum_x += p.first;
            sum_y += p.second;
        }

        if (valid_points == 0) return 0.0;

        double mean_angle = atan2(sum_y / valid_points, sum_x / valid_points);

        mean_angle += M_PI_2;

        while (mean_angle > M_PI) mean_angle -= 2 * M_PI;
        while (mean_angle < -M_PI) mean_angle += 2 * M_PI;

        return mean_angle;
    }

    bool toggleAlignment(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (req.data)
        {
            is_align_enabled_ = true;
            std::cout<<"Line Aligner with Hokuyo --- Start align with LaserScan."<<std::endl;

            while (ros::ok() && !isAligned())
            {
                process(ros::TimerEvent()); 
                ros::spinOnce();            
                ros::Duration(0.05).sleep();
            }

            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_.publish(cmd);

            is_align_enabled_ = false;
            res.success = true;
            res.message = "Line Aligner with Hokuyo --- Align done.";
        }
        else {
            is_align_enabled_ = false;
            res.success = true;
            res.message = "Line Aligner with Hokuyo --- Align disabled";
        }

        return true;
    }

    bool isAligned()
    {
        const double ALIGNED_THRESHOLD = 0.05;
        
        double angle_error = 0.0;
        if (use_ransac_) {
            angle_error = computeAngleErrorRANSAC(points_);
        } else {
            angle_error = computeAngleErrorCenterOfMass(points_);
        }

        return fabs(angle_error) < ALIGNED_THRESHOLD; 
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_aligner_service");
    LineAligner aligner;
    ros::spin();
    return 0;
}
