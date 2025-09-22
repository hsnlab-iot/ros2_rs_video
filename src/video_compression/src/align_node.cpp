#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

using std::placeholders::_1;

struct CameraIntrinsics {
    float fx, fy, cx, cy;
};

CameraIntrinsics load_intrinsics(const std::string& filename) {
    CameraIntrinsics intr;
    
    try {
        YAML::Node config = YAML::LoadFile(filename);
        
        // Try different possible structures for camera calibration YAML files
        if (config["camera_matrix"]) {
            // Standard OpenCV calibration format
            auto camera_matrix = config["camera_matrix"]["data"];
            intr.fx = camera_matrix[0].as<float>();
            intr.fy = camera_matrix[4].as<float>();
            intr.cx = camera_matrix[2].as<float>();
            intr.cy = camera_matrix[5].as<float>();
        }
        else if (config["camera_info"]) {
            // ROS camera_info format
            auto K = config["camera_info"]["K"];
            intr.fx = K[0].as<float>();
            intr.fy = K[4].as<float>();
            intr.cx = K[2].as<float>();
            intr.cy = K[5].as<float>();
        }
        else if (config["intrinsics"]) {
            // Custom intrinsics format
            intr.fx = config["intrinsics"]["fx"].as<float>();
            intr.fy = config["intrinsics"]["fy"].as<float>();
            intr.cx = config["intrinsics"]["cx"].as<float>();
            intr.cy = config["intrinsics"]["cy"].as<float>();
        }
        else if (config["fx"] && config["fy"] && config["cx"] && config["cy"]) {
            // Direct parameter format
            intr.fx = config["fx"].as<float>();
            intr.fy = config["fy"].as<float>();
            intr.cx = config["cx"].as<float>();
            intr.cy = config["cy"].as<float>();
        }
        else {
            throw std::runtime_error("Unknown YAML format for camera intrinsics");
        }
        
        RCLCPP_INFO(rclcpp::get_logger("align_node"), 
                   "Loaded intrinsics from %s: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                   filename.c_str(), intr.fx, intr.fy, intr.cx, intr.cy);
    }
    catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("align_node"), 
                    "Failed to load intrinsics from %s: %s", filename.c_str(), e.what());
        // Set default values if loading fails
        intr.fx = intr.fy = -1.0f;
        intr.cx = intr.cy = -1.0f;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("align_node"), 
                    "Error loading intrinsics from %s: %s", filename.c_str(), e.what());
        // Set default values if loading fails
        intr.fx = intr.fy = -1.0f;
        intr.cx = intr.cy = -1.0f;
    }
    
    return intr;
}

class AlignNode : public rclcpp::Node {
public:
    AlignNode()
    : Node("align_node")
    {
        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "color/image", 10, std::bind(&AlignNode::color_callback, this, _1));
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "depth/image", 10, std::bind(&AlignNode::depth_callback, this, _1));
        aligned_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aligned_depth/image", 10);

        // Load intrinsics and translation
        color_intr_ = load_intrinsics("calibration_color.yaml");
        depth_intr_ = load_intrinsics("calibration_depth.yaml");
        if (color_intr_.fx < 0 || depth_intr_.fx < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera intrinsics. Exiting.");
            rclcpp::shutdown();
        }

        this->declare_parameter<float>("tx", 1.0);
        this->declare_parameter<float>("ty", 0.0);
        this->declare_parameter<float>("tz", 0.0);

        tx_ = this->get_parameter("tx").as_double();
        ty_ = this->get_parameter("ty").as_double();
        tz_ = this->get_parameter("tz").as_double();
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

    void try_align() {
        if (!color_msg_ || !depth_msg_) return;

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