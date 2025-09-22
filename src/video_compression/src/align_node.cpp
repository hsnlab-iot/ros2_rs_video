#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>

using std::placeholders::_1;

struct CameraIntrinsics {
    float fx, fy, cx, cy;
    bool valid = false;
    
    void set_from_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr& info) {
        if (info && info->k.size() >= 9) {
            fx = info->k[0];  // K[0,0]
            fy = info->k[4];  // K[1,1]
            cx = info->k[2];  // K[0,2]
            cy = info->k[5];  // K[1,2]
            valid = true;
            RCLCPP_INFO(rclcpp::get_logger("align_node"), 
                       "Updated intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                       fx, fy, cx, cy);
        }
    }
};

class AlignNode : public rclcpp::Node {
public:
    AlignNode()
    : Node("align_node")
    {
        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "color/image_raw", 10, std::bind(&AlignNode::color_callback, this, _1));
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "depth/image_raw", 10, std::bind(&AlignNode::depth_callback, this, _1));
        
        // Subscribe to camera_info topics
        color_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "color/camera_info", 10, std::bind(&AlignNode::color_info_callback, this, _1));
        depth_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "depth/camera_info", 10, std::bind(&AlignNode::depth_info_callback, this, _1));
            
        aligned_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aligned_depth/image", 10);

        this->declare_parameter<float>("tx", 1.0);
        this->declare_parameter<float>("ty", 0.0);
        this->declare_parameter<float>("tz", 0.0);

        tx_ = this->get_parameter("tx").as_double();
        ty_ = this->get_parameter("ty").as_double();
        tz_ = this->get_parameter("tz").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Align node started. Waiting for camera_info messages...");
    }

private:
    void color_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        color_msg_ = msg;
        try_align();
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        depth_msg_ = msg;
        try_align();
    }
    
    void color_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        color_intr_.set_from_camera_info(msg);
    }
    
    void depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        depth_intr_.set_from_camera_info(msg);
    }

    void try_align() {
        if (!color_msg_ || !depth_msg_) return;
        
        // Check if we have valid intrinsics
        if (!color_intr_.valid || !depth_intr_.valid) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "Waiting for camera intrinsics. Color valid: %s, Depth valid: %s",
                                 color_intr_.valid ? "true" : "false",
                                 depth_intr_.valid ? "true" : "false");
            return;
        }

        cv_bridge::CvImagePtr cv_color = cv_bridge::toCvCopy(color_msg_, "bgr8");
        cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_msg_, "16UC1");

        int width = cv_color->image.cols;
        int height = cv_color->image.rows;

        cv::Mat aligned_depth(height, width, CV_16UC1, cv::Scalar(0));

        // For each color pixel, find corresponding depth
        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                // Backproject color pixel to 3D (in color cam)
                float x_c = (u - color_intr_.cx) / color_intr_.fx;
                float y_c = (v - color_intr_.cy) / color_intr_.fy;

                // Assume unit depth, transform to depth cam
                Eigen::Vector3f pt_c(x_c, y_c, 1.0f);
                Eigen::Vector3f pt_d = pt_c + Eigen::Vector3f(tx_, ty_, tz_);

                // Project to depth image
                float u_d = pt_d.x() * depth_intr_.fx / pt_d.z() + depth_intr_.cx;
                float v_d = pt_d.y() * depth_intr_.fy / pt_d.z() + depth_intr_.cy;

                int u_di = static_cast<int>(u_d);
                int v_di = static_cast<int>(v_d);

                if (u_di >= 0 && u_di < cv_depth->image.cols &&
                    v_di >= 0 && v_di < cv_depth->image.rows) {
                    uint16_t depth_val = cv_depth->image.at<uint16_t>(v_di, u_di);
                    aligned_depth.at<uint16_t>(v, u) = depth_val;
                }
            }
        }

        auto out_msg = cv_bridge::CvImage(color_msg_->header, "16UC1", aligned_depth).toImageMsg();
        aligned_depth_pub_->publish(*out_msg);

        // Reset messages
        color_msg_.reset();
        depth_msg_.reset();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aligned_depth_pub_;

    sensor_msgs::msg::Image::SharedPtr color_msg_;
    sensor_msgs::msg::Image::SharedPtr depth_msg_;

    CameraIntrinsics color_intr_, depth_intr_;
    float tx_, ty_, tz_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlignNode>());
    rclcpp::shutdown();
    return 0;
}