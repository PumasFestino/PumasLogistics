#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <vector>
#include <cmath>
#include <random>
#include <limits>
#include <algorithm>

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

    enum State { IDLE, ALIGNING, CENTERING };
    State state_;

    const int MAX_ITER = 100;
    const double DIST_THRESHOLD = 0.05;

public:
    LineAligner() : state_(IDLE)
    {
        scan_sub_ = nh_.subscribe("/scan", 1, &LineAligner::scanCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        service_ = nh_.advertiseService("/navigation/align_with_line", &LineAligner::toggleAlignment, this);

        is_align_enabled_ = false;
        use_ransac_ = true;
        timer_ = nh_.createTimer(ros::Duration(0.05), &LineAligner::process, this);

        ROS_INFO("Line Aligner with Hokuyo --- Soft by Joshua M");
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        points_.clear();
        for (size_t i = 0; i < msg->ranges.size(); ++i)
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

    double computeLineCenterOffset()
    {
        // 1. Filtrar puntos en frente del robot
        std::vector<double> y_values;
        for (const auto& p : points_) {
            // Considerar solo puntos en un rango frontal (x > 0) y a distancia razonable
            if (p.first > 0.1 && p.first < 2.0) {
                y_values.push_back(p.second);
            }
        }
        
        if (y_values.size() < 10) {
            ROS_WARN_THROTTLE(1.0, "Not enough points for center calculation");
            return 0.0;
        }
        
        // 2. Ordenar para encontrar los extremos de la línea
        std::sort(y_values.begin(), y_values.end());
        
        // 3. Calcular centro usando percentiles robustos (ignorar outliers)
        const double lower_percentile = 0.15;
        const double upper_percentile = 0.85;
        
        int lower_idx = static_cast<int>(y_values.size() * lower_percentile);
        int upper_idx = static_cast<int>(y_values.size() * upper_percentile);
        
        double left_edge = y_values[lower_idx];   // Lado "izquierdo" de la línea
        double right_edge = y_values[upper_idx];  // Lado "derecho" de la línea
        
        // 4. Calcular centro geométrico de la línea
        double line_center = (left_edge + right_edge) / 2.0;
        
        ROS_DEBUG_THROTTLE(0.5, "Line edges: left=%.3f, right=%.3f, center=%.3f", 
                          left_edge, right_edge, line_center);
        
        return line_center;
    }

    void process(const ros::TimerEvent&)
    {
        if (!is_align_enabled_ || points_.size() < 2) return;

        switch(state_) {
            case ALIGNING: {
                double angle_error = use_ransac_ ? computeAngleErrorRANSAC(points_) 
                                                : computeAngleErrorCenterOfMass(points_);
                cmd.angular.z = -0.5 * angle_error;
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd_pub_.publish(cmd);
                break;
            }
            case CENTERING: {
                double center_error = computeLineCenterOffset();
                
                double kp = -0.5;  // Ganancia para movimiento lateral
                cmd.linear.y = -kp * center_error;
                cmd.angular.z = 0.0;
                cmd.linear.x = 0.0;
                cmd_pub_.publish(cmd);
                
                ROS_INFO_STREAM_THROTTLE(0.5, "Center error: " << center_error);
                break;
            }
            case IDLE:
                break;
        }
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
            timer_.stop();
            is_align_enabled_ = true;
            ROS_INFO("Line Aligner with Hokuyo --- Start align with LaserScan.");

            // Fase de alineación angular
            state_ = ALIGNING;
            while (ros::ok() && !isAligned())
            {
                process(ros::TimerEvent());
                ros::spinOnce();
                ros::Duration(0.05).sleep();
            }

            // Detener rotación
            /*cmd = geometry_msgs::Twist();
            cmd_pub_.publish(cmd);

            // Fase de centrado en la línea
            state_ = CENTERING;
            while (ros::ok() && !isCentered())
            {
                process(ros::TimerEvent());
                ros::spinOnce();
                ros::Duration(0.05).sleep();
            }

            // Detener completamente*/
            cmd = geometry_msgs::Twist();
            cmd_pub_.publish(cmd);
            
            is_align_enabled_ = false;
            state_ = IDLE;
            timer_.start();

            res.success = true;
            res.message = "Line Aligner with Hokuyo --- Align done.";
        }
        else {
            is_align_enabled_ = false;
            state_ = IDLE;
            res.success = true;
            res.message = "Line Aligner with Hokuyo --- Align disabled";
        }
        return true;
    }

    bool isAligned()
    {
        const double ALIGNED_THRESHOLD = 0.04; // ~3°
        double angle_error = use_ransac_ ? computeAngleErrorRANSAC(points_) 
                                        : computeAngleErrorCenterOfMass(points_);
        return fabs(angle_error) < ALIGNED_THRESHOLD; 
    }

    bool isCentered()
    {
        const double CENTERED_THRESHOLD = 0.07; // 2 cm
        double center_error = computeLineCenterOffset();
        return fabs(center_error) < CENTERED_THRESHOLD;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_aligner_service");
    LineAligner aligner;
    ros::spin();
    return 0;
}