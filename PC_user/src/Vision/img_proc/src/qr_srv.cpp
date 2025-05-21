#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <img_proc/ReadQRCode.h>

class QRCodeReader
{
public:
    QRCodeReader()
    {
        image_sub_ = nh_.subscribe("/camera/rgb/image_color", 1, &QRCodeReader::imageCallback, this);
        service_ = nh_.advertiseService("/vision/read_qr_code", &QRCodeReader::serviceCallback, this);
        latest_image_received_ = false;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::ServiceServer service_;
    cv::Mat latest_image_;
    bool latest_image_received_;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            latest_image_ = cv_ptr->image.clone();
            latest_image_received_ = true;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            latest_image_received_ = false;
        }
    }

    bool serviceCallback(img_proc::ReadQRCode::Request& req, img_proc::ReadQRCode::Response& res)
    {
        if (!req.enabled)
        {
            ROS_WARN("Service called with trigger = false. Doing nothing.");
            res.success = false;
            res.qr_data = "";
            return true;
        }

        if (!latest_image_received_)
        {
            ROS_WARN("No image received yet.");
            res.success = false;
            res.qr_data = "";
            return true;
        }

        // Convertir la imagen a escala de grises
        cv::Mat gray;
        cv::cvtColor(latest_image_, gray, cv::COLOR_BGR2GRAY);

        // Crear el escáner de QR
        zbar::ImageScanner scanner;
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

        // Convertir la imagen OpenCV a formato compatible con ZBar
        zbar::Image zbarImage(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);

        // Escanear la imagen
        scanner.scan(zbarImage);

        // Imprimir los resultados
        for (auto symbol = zbarImage.symbol_begin(); symbol != zbarImage.symbol_end(); ++symbol)
        {
            res.qr_data = symbol->get_data();
            res.success = true;
            ROS_INFO("QR Code detected: %s", res.qr_data.c_str());
            return true;
        }

        // Si no se encuentra ningún código QR
        ROS_INFO("No QR Code detected.");
        res.success = false;
        res.qr_data = "";
        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_code_reader");
    QRCodeReader reader;
    ros::spin();
    return 0;
}
