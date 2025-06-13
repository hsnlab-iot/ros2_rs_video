#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <zmq.h>
#include <thread>
#include <cstring>
#include <rclcpp/publisher_options.hpp>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <memory>
#include <string>
#include <vector>

class ZMQGStreamerPublisher : public rclcpp::Node {
public:
    ZMQGStreamerPublisher()
    : Node("zmq_h264_gstreamer_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ZMQ GStreamer Publisher Node");

        zmq_url_ = this->declare_parameter<std::string>("zmq_url", "ipc:///tmp/image");
        mode_ = this->declare_parameter<std::string>("mode", "color");
        width_ = this->declare_parameter<int>("width", 1280);
        height_ = this->declare_parameter<int>("height", 720);
        camera_info_file_ = this->declare_parameter<std::string>("camera_info_file", "");
        frame_name_ = declare_parameter<std::string>("frame_name", "camera");

        if (mode_ != "color" && mode_ != "depth") {
            RCLCPP_ERROR(this->get_logger(), "Invalid mode specified: %s. Supported modes are 'color' and 'depth'.", mode_.c_str());
            throw std::runtime_error("Invalid mode specified.");
        }

        // Allow QoS overrides for depth, reliability, durability
        rclcpp::QoS qos(rclcpp::KeepLast(3));
        rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> pub_options_i;
        pub_options_i.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_raw", qos, pub_options_i);

        if (!camera_info_file_.empty()) {
            if (!load_camera_info(camera_info_file_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load camera info from %s", camera_info_file_.c_str());
                throw std::runtime_error("Camera info file not found");
            } else {
                RCLCPP_INFO(this->get_logger(), "Loaded camera info from %s", camera_info_file_.c_str());
            }
        }

        zmq_thread_ = std::thread(&ZMQGStreamerPublisher::zmq_listener, this);

        if (camera_info_) {
            rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> pub_options_ci;
            pub_options_ci.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
            camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
                "camera_info", qos, pub_options_ci);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), std::bind(&ZMQGStreamerPublisher::publish_camera_info, this));
            RCLCPP_INFO(this->get_logger(), "Camera info publisher running.");
        }
    }

    ~ZMQGStreamerPublisher() override {
        if (zmq_thread_.joinable()) {
            zmq_thread_.join();
        }
    }

private:
    std::string zmq_url_, mode_;
    int width_, height_;
    std::thread zmq_thread_;
    size_t zmq_msg_count_ = 0; // ZMQ message counter
    size_t ros_msg_count_ = 0; // ROS message counter
    std::string camera_info_file_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    std::string frame_name_;
    rclcpp::TimerBase::SharedPtr timer_;

    void zmq_listener() {
        RCLCPP_INFO(this->get_logger(), "Listening for ZMQ messages on %s", zmq_url_.c_str());
        void* context = zmq_ctx_new();
        void* socket = zmq_socket(context, ZMQ_SUB);
        zmq_connect(socket, zmq_url_.c_str());
        int conflate = 1;
        zmq_setsockopt(socket, ZMQ_CONFLATE, &conflate, sizeof(conflate));
        zmq_setsockopt(socket, ZMQ_SUBSCRIBE, "", 0);

        while (rclcpp::ok()) {
            zmq_msg_t msg;
            zmq_msg_init(&msg);
            int rc = zmq_msg_recv(&msg, socket, 0);
            if (rc >= 0) {
                size_t size = zmq_msg_size(&msg);
                void* data = zmq_msg_data(&msg);
                ++zmq_msg_count_;
                if (zmq_msg_count_ % 100 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Received %zu ZMQ messages. Actual size: %ld", zmq_msg_count_, size);
                }

                if (mode_ == "color") {
                    if (size != width_ * height_ * 3) {
                        RCLCPP_ERROR(this->get_logger(), "Received data size %ld does not match expected size %d for color mode.", size, width_ * height_ * 3);
                        continue;
                    }
                    auto msg = std::make_unique<sensor_msgs::msg::Image>();
                    msg->header.stamp = this->now();
                    msg->header.frame_id = "camera";
                    msg->height = height_;
                    msg->width = width_;
                    msg->encoding = "rgb8";
                    msg->is_bigendian = false;
                    msg->step = width_ * 3;
                    msg->data.assign(static_cast<uint8_t*>(data), static_cast<uint8_t*>(data) + size);
                    publisher_->publish(std::move(msg));
                    ++ros_msg_count_;
                }
                else if (mode_ == "depth") {
                    if (size != width_ * height_ * 2) {
                        RCLCPP_ERROR(this->get_logger(), "Received data size %ld does not match expected size %d for depth mode.", size, width_ * height_ * 2);
                        continue;
                    }
                    auto msg = std::make_unique<sensor_msgs::msg::Image>();
                    msg->header.stamp = this->now();
                    msg->header.frame_id = "camera";
                    msg->height = height_;
                    msg->width = width_;
                    msg->encoding = "16UC1"; // 16-bit unsigned single-channel
                    msg->is_bigendian = false;
                    msg->step = width_ * 2; // 2 bytes per pixel for 16-bit depth
                    msg->data.assign(static_cast<uint8_t*>(data), static_cast<uint8_t*>(data) + size);
                    publisher_->publish(std::move(msg));
                    ++ros_msg_count_;
                }
                if (ros_msg_count_ % 100 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Published %zu ROS messages.", ros_msg_count_);
                }
            }
            zmq_msg_close(&msg);
        }
        zmq_close(socket);
        zmq_ctx_term(context);
    }

    // Camera info loader
    bool load_camera_info(const std::string& file_path) {
        try {
            YAML::Node calib = YAML::LoadFile(file_path);
            camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
            camera_info_->width = calib["image_width"].as<int>();
            camera_info_->height = calib["image_height"].as<int>();

            // Copy K (3x3)
            auto k_vec = calib["camera_matrix"]["data"].as<std::vector<double>>();
            std::copy_n(k_vec.begin(), std::min<size_t>(k_vec.size(), camera_info_->k.size()), camera_info_->k.begin());

            // Copy D (variable length)
            camera_info_->d = calib["distortion_coefficients"]["data"].as<std::vector<double>>();

            // Copy R (3x3)
            auto r_vec = calib["rectification_matrix"]["data"].as<std::vector<double>>();
            std::copy_n(r_vec.begin(), std::min<size_t>(r_vec.size(), camera_info_->r.size()), camera_info_->r.begin());

            // Copy P (3x4)
            auto p_vec = calib["projection_matrix"]["data"].as<std::vector<double>>();
            std::copy_n(p_vec.begin(), std::min<size_t>(p_vec.size(), camera_info_->p.size()), camera_info_->p.begin());

            camera_info_->distortion_model = calib["distortion_model"].as<std::string>();
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading camera info: %s", e.what());
            return false;
        }
    }

    void publish_camera_info() {
        if (!camera_info_) return;
        camera_info_->header.stamp = this->now();
        camera_info_->header.frame_id = frame_name_;
        camera_info_pub_->publish(*camera_info_);
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZMQGStreamerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}