#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <random>

class RandomImagePublisher : public rclcpp::Node {
public:
    RandomImagePublisher()
    : Node("random_image_publisher")
    {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/video/color/image_raw", rclcpp::SensorDataQoS());

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // ~30 FPS
            std::bind(&RandomImagePublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        // Create random image
        cv::Mat img(480, 640, CV_8UC3);
        cv::randu(img, cv::Scalar::all(0), cv::Scalar::all(255));

        // Convert to ROS2 Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera";

        image_pub_->publish(*msg);

        static size_t count = 0;
        count++;
        if (count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Published %zu images", count);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RandomImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}