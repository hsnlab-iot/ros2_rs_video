#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <fstream>
#include <string>
#include <vector>

class GStreamerDecoder : public rclcpp::Node {
public:
    GStreamerDecoder()
    : Node("gstreamer_decoder_node")
    {
        width_ = declare_parameter<int>("width", 1280);
        height_ = declare_parameter<int>("height", 720);
        rate_ = declare_parameter<int>("rate", 30);
        frame_name_ = declare_parameter<std::string>("frame_name", "camera");
        camera_info_file_ = declare_parameter<std::string>("camera_info_file", "");
        mode_ = declare_parameter<std::string>("mode", "color");

        // Initialize GStreamer
        gst_init(nullptr, nullptr);

        if (mode_ != "color" && mode_ != "depth_rgb" && mode_ != "depth_yuv" && mode_ != "depth_yuv_12") {
            RCLCPP_ERROR(this->get_logger(), "Invalid mode specified: %s", mode_.c_str());
            throw std::runtime_error("Invalid mode specified.");
        }

        if (mode_ == "color" || mode_ == "depth_rgb") {
            format_ = "RGB"; // RGB format for color and depth RGB
        } else if (mode_ == "depth_yuv") {
            format_ = "Y444"; // YUV format for depth
        } else if (mode_ == "depth_yuv_12") {
            format_ = "Y444_12LE"; // Grayscale format
        }

        // Build GStreamer pipeline
        /*
        std::string pipeline_desc =
            "appsrc name=src is-live=true format=time ! "
            //"video/x-h264, stream-format=byte-stream, alignment=nal, profile=main ! "
            "h264parse ! avdec_h264 ! videoconvert ! "
            "video/x-raw, format=RGB ! "
            "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true";
        */
        std::ostringstream pipeline_ss;
        pipeline_ss
            << "appsrc name=src is-live=true format=time ! "
            "h265parse ! avdec_h265 ! videoconvert ! "
            "video/x-raw, format=" << format_ << " ! "
            "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true";
        
        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_ss.str().c_str(), &error);
        if (!pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer pipeline: %s", error->message);
            g_error_free(error);
            throw std::runtime_error("Failed to create GStreamer pipeline");
        }

        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");

        // Set caps on appsrc
        /*
        std::string caps_str = "video/x-h264, stream-format=byte-stream, alignment=nal, profile=main, width=" +
            std::to_string(width_) + ", height=" + std::to_string(height_) + ", framerate=" + std::to_string(rate_) + "/1";
        */
        std::string caps_str = "video/x-h265, stream-format=byte-stream, alignment=au, width=" +
            std::to_string(width_) + ", height=" + std::to_string(height_) + ", framerate=" + std::to_string(rate_) + "/1";
        GstCaps* caps = gst_caps_from_string(caps_str.c_str());
        g_object_set(G_OBJECT(appsrc_), "caps", caps, nullptr);
        gst_caps_unref(caps);

        // Connect new-sample signal
        g_signal_connect(appsink_, "new-sample", G_CALLBACK(&GStreamerDecoder::on_new_sample_static), this);

        // Publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/video/image_raw", rclcpp::SensorDataQoS());
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/video/camera_info", 3);

        // Load camera info if provided
        if (!camera_info_file_.empty()) {
            if (!load_camera_info(camera_info_file_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load camera info from %s", camera_info_file_.c_str());
                throw std::runtime_error("Camera info file not found");
            }
        }

        // Subscription
        compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/video/h264", rclcpp::SensorDataQoS(),
            std::bind(&GStreamerDecoder::compressed_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline initialized.");
    }

    ~GStreamerDecoder() {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
        if (appsrc_) gst_object_unref(appsrc_);
        if (appsink_) gst_object_unref(appsink_);
    }

    void start() {
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline started.");

        if (camera_info_) {
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&GStreamerDecoder::publish_camera_info, this)
            );
        }
    }

private:
    // Parameters
    int width_, height_, rate_;
    std::string frame_name_;
    std::string camera_info_file_;
    std::string mode_, format_;

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_;

    // GStreamer
    GstElement* pipeline_ = nullptr;
    GstElement* appsrc_ = nullptr;
    GstElement* appsink_ = nullptr;

    // cv_bridge
    cv_bridge::CvImage cv_image_;

    // Message counters
    size_t compressed_counter_ = 0;
    size_t raw_counter_ = 0;

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

    void compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        // Push H.264 bytes to appsrc
        compressed_counter_++;
        if (compressed_counter_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Compressed input messages received: %zu", compressed_counter_);
        }
        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
        gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());
        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);
    }

    static GstFlowReturn on_new_sample_static(GstAppSink* sink, gpointer user_data) {
        return static_cast<GStreamerDecoder*>(user_data)->on_new_sample(sink);
    }

    GstFlowReturn on_new_sample(GstAppSink* sink) {
        GstSample* sample = gst_app_sink_pull_sample(sink);
        if (!sample) return GST_FLOW_ERROR;

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstCaps* caps = gst_sample_get_caps(sample);
        GstStructure* s = gst_caps_get_structure(caps, 0);
        int width, height;
        gst_structure_get_int(s, "width", &width);
        gst_structure_get_int(s, "height", &height);

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        try {
            cv::Mat frame(height, width, CV_8UC3, (void*)map.data);
            sensor_msgs::msg::Image::SharedPtr ros_img;
            if (mode_ == "color") {
                ros_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            } else if (mode_ == "depth_rgb") {
                // Convert RGB back to 16-bit depth
                cv::Mat depth_frame(height, width, CV_16UC1);
                #pragma omp parallel for
                for (int v = 0; v < height; ++v) {
                    for (int u = 0; u < width; ++u) {
                        const cv::Vec3b& rgb = frame.at<cv::Vec3b>(v, u);
                        uint8_t r4 = rgb[0] >> 4;
                        uint8_t g4 = rgb[1] >> 4;
                        uint8_t b4 = rgb[2] >> 4;
                        uint16_t depth_value = (r4 << 8) | (g4 << 4) | b4;
                        depth_frame.at<uint16_t>(v, u) = depth_value;
                    }
                }
                ros_img = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth_frame).toImageMsg();
            } else if (mode_ == "depth_yuv") {
                // Convert YUV back to 16-bit depth
                cv::Mat depth_frame(height, width, CV_16UC1);
                uint8_t* y = map.data;
                uint8_t* u = y + width * height;
                uint8_t* v = u + width * height;
                #pragma omp parallel for
                for (int py = 0; py < height; ++py) {
                    for (int px = 0; px < width; ++px) {
                        uint8_t yv = y[py * width + px];
                        uint8_t uv = u[py * width + px]; // U
                        uint8_t vv = v[py * width + px]; // V
                        // Convert YUV to 16-bit depth value
                        // Y: d11, d10, d9, d8, d5, d4, x/(d3), x/(d2)
                        uint16_t d = (((uint16_t)yv) << 4) & 0xf00; // d11, d10, d9, d8
                        d |= (yv << 2) & 0x30; // d5, d4
                        // U: d7, d3, d1, x, x, x, x, x
                        d |= (uv & 0x80) | ((uv >> 3) & 0x08) | ((uv >> 4) & 0x02); // d7, d3, d1
                        // V: d6, d2, d0, x, x, x, x, x
                        d |= (vv >> 1 & 0x40) | ((vv >> 4) & 0x04) | ((vv >> 5) & 0x01); // d6, d2, d0                        
                        depth_frame.at<uint16_t>(py, px) = d;
                    }
                }
                ros_img = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth_frame).toImageMsg();
            } else if (mode_ == "depth_yuv_12") {
                cv::Mat gray_frame(height, width, CV_16UC1, (void*)map.data);
                ros_img = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", gray_frame).toImageMsg();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unsupported mode: %s", mode_.c_str());
                throw std::runtime_error("Unsupported mode in on_new_sample");
            }

            ros_img->header.stamp = this->now();
            ros_img->header.frame_id = frame_name_;
            image_pub_->publish(*ros_img);

            raw_counter_++;
            if (raw_counter_ % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), "Raw output messages published: %zu", raw_counter_);
            }
        } catch (...) {
            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GStreamerDecoder>();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}