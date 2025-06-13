#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <zmq.h>
#include <thread>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <cstring>

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
        rate_ = this->declare_parameter<int>("rate", 30);
        bitrate_ = this->declare_parameter<int>("bitrate", 2000);

        if (mode_ != "color" && mode_ != "depth_rgb" && mode_ != "depth_yuv" && mode_ != "depth_yuv_12" && mode_ != "gray") {
            RCLCPP_ERROR(this->get_logger(), "Invalid mode specified: %s. Supported modes are: color, depth_rgb, depth_yuv, depth_yuv_12", mode_.c_str());
            throw std::runtime_error("Invalid mode specified.");
        }

        if (mode_ == "color") {
            format_ = "I420";
        } else if (mode_ == "depth_rgb") {
            format_ = "I420";
        } else if (mode_ == "depth_yuv") {
            format_ = "Y444";
        } else if (mode_ == "depth_yuv_12") {
            format_ = "Y444_12LE";
        }

        gst_init(nullptr, nullptr);

        /*
        std::ostringstream pipeline_ss;
        pipeline_ss
            << "appsrc name=src ! "
            << "videoconvert ! "
            << "video/x-raw,format=Y444,width=" << width_ << ",height=" << height_ << ",framerate=" << rate_ << "/1 ! "
            << "x264enc tune=zerolatency bitrate=" << bitrate_ << " speed-preset=ultrafast b-adapt=false key-int-max=3 sliced-threads=true byte-stream=true ! "
            //<< "video/x-h264,profile=main,stream-format=byte-stream,alignment=nal ! "
            << "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true";
        */

        std::ostringstream pipeline_ss;
        pipeline_ss
            << "appsrc name=src ! videoconvert ! "
            << "video/x-raw,format=" << format_ << ",width=" << width_ << ",height=" << height_ << ",framerate=" << rate_ << "/1 ! "
            << "x265enc tune=zerolatency bitrate=" << bitrate_ << " speed-preset=ultrafast key-int-max=15 ! "
            << "video/x-h265,stream-format=byte-stream,alignment=au ! "
            << "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true";

        pipeline_ = gst_parse_launch(pipeline_ss.str().c_str(), nullptr);
        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        
        if (!appsrc_ || !appsink_) {
            RCLCPP_ERROR(this->get_logger(), "GStreamer pipeline must contain named 'appsrc' and 'appsink' elements.");
            throw std::runtime_error("Invalid GStreamer pipeline.");
        }

        if (format_ == "I420") {
            format_ = "RGB";
        }

        GstCaps* caps = gst_caps_new_simple(
            "video/x-raw",
            "format", G_TYPE_STRING, format_.c_str(),
            "width", G_TYPE_INT, width_,
            "height", G_TYPE_INT, height_,
            "framerate", GST_TYPE_FRACTION, rate_, 1,
            NULL
        );
        g_object_set(appsrc_, "caps", caps, NULL);
        gst_caps_unref(caps);

        g_signal_connect(appsink_, "new-sample", G_CALLBACK(&ZMQGStreamerPublisher::on_new_sample_static), this);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("video/h264", 3);

        zmq_thread_ = std::thread(&ZMQGStreamerPublisher::zmq_listener, this);
    }

    ~ZMQGStreamerPublisher() override {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(appsrc_);
        gst_object_unref(appsink_);
        gst_object_unref(pipeline_);
        if (zmq_thread_.joinable()) {
            zmq_thread_.join();
        }
    }

private:
    std::string zmq_url_, mode_, format_;
    int width_, height_, rate_, bitrate_;
    GstElement *pipeline_{nullptr}, *appsrc_{nullptr}, *appsink_{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    std::thread zmq_thread_;
    size_t zmq_msg_count_ = 0; // ZMQ message counter
    size_t ros_msg_count_ = 0; // ROS message counter

    std::vector<std::vector<uint8_t>> parameter_sets;   // VPS, SPS, PPS parameter sets
    bool extracted_parameter_sets = false;

    void zmq_listener() {
        RCLCPP_INFO(this->get_logger(), "Listening for ZMQ messages on %s", zmq_url_.c_str());
        void* context = zmq_ctx_new();
        void* socket = zmq_socket(context, ZMQ_SUB);
        zmq_connect(socket, zmq_url_.c_str());
        zmq_setsockopt(socket, ZMQ_SUBSCRIBE, "", 0);

        uint8_t* depth_rgb = nullptr;
        uint8_t* depth_yuv = nullptr;
        uint16_t* depth_yuv_12 = nullptr;
        if (mode_ == "depth_rgb") {
            depth_rgb = new uint8_t[width_ * height_ * 3];
        }
        else if (mode_ == "depth_yuv") {
            depth_yuv = new uint8_t[width_ * height_ * 3];
        }
        else if (mode_ == "depth_yuv_12") {
            depth_yuv_12 = new uint16_t[width_ * height_ * 3] ;
        }

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

                if (mode_ == "depth_rgb") {
                    if (size != width_ * height_ * 2) {
                        RCLCPP_ERROR(this->get_logger(), "Received depth data size mismatch: expected %d, got %ld", width_ * height_ * 2, size);
                        continue;
                    }
                    // Convert depth data to RGB
                    uint8_t* ddata = static_cast<uint8_t*>(zmq_msg_data(&msg));
                    #pragma omp parallel for
                    for (int i = 0; i < width_ * height_; ++i) {
                        uint16_t depth_value = static_cast<uint16_t>(ddata[i * 2] | (ddata[i * 2 + 1] << 8));
                        uint8_t r4 = (depth_value >> 8) & 0xF;
                        uint8_t g4 = (depth_value >> 4) & 0xF;
                        uint8_t b4 = depth_value & 0xF;
                        depth_rgb[i * 3]     = r4 << 4;
                        depth_rgb[i * 3 + 1] = g4 << 4;
                        depth_rgb[i * 3 + 2] = b4 << 4;
                    }
                    data = static_cast<void*>(depth_rgb);
                    size = width_ * height_ * 3;
                }
                else if (mode_ == "depth_yuv") {
                    if (size != width_ * height_ * 2) {
                        RCLCPP_ERROR(this->get_logger(), "Received depth data size mismatch: expected %d, got %ld", width_ * height_ * 2, size);
                        continue;
                    }
                    // Convert depth data to YUV
                    uint8_t* ddata = static_cast<uint8_t*>(zmq_msg_data(&msg));
                    #pragma omp parallel for
                    for (int i = 0; i < width_ * height_; ++i) {
                        uint16_t d = static_cast<uint16_t>(ddata[i * 2] | (ddata[i * 2 + 1] << 8));
                        // Y: d11, d10, d9, d8, d5, d4, x/(d3), x/(d2)
                        uint8_t y = ((d >> 4) & 0xf0) | ((d >> 2) & 0x0f);
                        // U: d7, d3, d1, x, x, x, x, x
                        uint8_t u = ((d >> 5) & 0x4) |
                                    ((d >> 2) & 0x2) |
                                    ((d >> 1) & 0x1);
                        // V: d6, d2, d0, x, x, x, x, x
                        uint8_t v = ((d >> 4) & 0x4) |
                                    ((d >> 1) & 0x2) |
                                    ((d) & 0x1);
                        depth_yuv[i] = y;
                        depth_yuv[i + width_ * height_] = u << 5;
                        depth_yuv[i + 2 * (width_ * height_)] = v << 5;
                    }
                    data = static_cast<void*>(depth_yuv);
                    size = width_ * height_ * 3;
                }
                else if (mode_ == "depth_yuv_12") {
                    if (size != width_ * height_ * 2) {
                        RCLCPP_ERROR(this->get_logger(), "Received depth data size mismatch: expected %d, got %ld", width_ * height_ * 2, size);
                        continue;
                    }
                    // Copy depth data to depth_yuv_12
                    std::memcpy(depth_yuv_12, data, width_ * height_ * 2);
                    // Fill U and V planes with 0x800
                    std::fill(depth_yuv_12 + width_ * height_, depth_yuv_12 + 3 * width_ * height_, 0x800);
                    data = static_cast<void*>(depth_yuv_12);
                    size = width_ * height_ * 3 * 2;
                }
                GstBuffer *buf = gst_buffer_new_allocate(nullptr, size, nullptr);
                gst_buffer_fill(buf, 0, data, size);
                GstFlowReturn ret;
                g_signal_emit_by_name(appsrc_, "push-buffer", buf, &ret);
                gst_buffer_unref(buf);
            } else {
                RCLCPP_ERROR(this->get_logger(), "ZMQ error: %s", zmq_strerror(zmq_errno()));
            }
            zmq_msg_close(&msg);
        }
        zmq_close(socket);
        zmq_ctx_term(context);
        if (depth_rgb) {
            delete[] depth_rgb;
        }
        if (depth_yuv) {
            delete[] depth_yuv;
        }
        if (depth_yuv_12) {
            delete[] depth_yuv_12;
        }
    }

    static GstFlowReturn on_new_sample_static(GstAppSink *sink, gpointer user_data) {
        return static_cast<ZMQGStreamerPublisher*>(user_data)->on_new_sample(sink);
    }

    GstFlowReturn on_new_sample(GstAppSink *sink) {
        GstSample *sample = gst_app_sink_pull_sample(sink);
        if (!sample) return GST_FLOW_ERROR;
        GstBuffer *buf = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

         // Extract VPS/SPS/PPS once
        if (!extracted_parameter_sets) {
            extract_vps_sps_pps(map.data, map.size, parameter_sets);
            if (!parameter_sets.empty()) {
                extracted_parameter_sets = true;
            }
        }

        auto nal_types = list_nal_unit_types_in_prefix(map.data, map.size);
        bool has_vps_sps_pps = nal_types.end() != std::find(nal_types.begin(), nal_types.end(), 32) ||
                               nal_types.end() != std::find(nal_types.begin(), nal_types.end(), 33) ||
                               nal_types.end() != std::find(nal_types.begin(), nal_types.end(), 34);
        bool keyframe = nal_types.end() != std::find(nal_types.begin(), nal_types.end(), 19) ||
                        nal_types.end() != std::find(nal_types.begin(), nal_types.end(), 20) ||
                        nal_types.end() != std::find(nal_types.begin(), nal_types.end(), 21);
        
        /* DEBIG only
        std::ostringstream nal_types_ss;
        for (size_t i = 0; i < nal_types.size(); ++i) {
            nal_types_ss << static_cast<int>(nal_types[i]);
            if (i + 1 < nal_types.size()) nal_types_ss << ",";
        }
        RCLCPP_INFO(this->get_logger(), "NAL types found: [%s], Keyframe: %s, VPS/SPS/PPS: %s",
            nal_types_ss.str().c_str(),
            keyframe ? "true" : "false",
            has_vps_sps_pps ? "true" : "false");
        */

        auto msg = sensor_msgs::msg::CompressedImage();
        msg.header.stamp = this->now();
        msg.format = "h265";
    
        std::vector<uint8_t> new_data;
        if (keyframe && extracted_parameter_sets && !has_vps_sps_pps) {
            for (const auto& nal : parameter_sets) {
                new_data.insert(new_data.end(), nal.begin(), nal.end());
            }
            new_data.insert(new_data.end(), map.data, map.data + map.size);
            msg.data = std::move(new_data);
            RCLCPP_DEBUG(this->get_logger(), "Keyframe detected, prepending VPS/SPS/PPS to message");
        } else {
            msg.data.assign(map.data, map.data + map.size);
        }
        publisher_->publish(msg);

        ++ros_msg_count_;
        if (ros_msg_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Published %zu ROS messages", ros_msg_count_);
        }

        gst_buffer_unmap(buf, &map);
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    bool extract_vps_sps_pps(const uint8_t* data, size_t size, std::vector<std::vector<uint8_t>>& out) {
        const uint8_t* p = data;
        const uint8_t* end = data + size;

        while (p + 4 < end) {
            // Look for start code (0x000001 or 0x00000001)
            if (p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x01) {
                const uint8_t* nal_start = p + 4;
                p += 4;

                if (p >= end) break;

                // NAL header: forbidden_zero(1) + nal_unit_type(6) + layer_id(6) + tid(3)
                uint8_t nal_unit_type = (nal_start[0] >> 1) & 0x3F;

                const uint8_t* nal_end = std::find(p, end - 4, 0x00);
                while (nal_end < end - 4 && !(nal_end[1] == 0x00 && nal_end[2] == 0x00 && nal_end[3] == 0x01)) {
                    nal_end = std::find(nal_end + 1, end - 4, 0x00);
                }

                size_t nal_size = (nal_end > p) ? (nal_end - p) : (end - p);
                std::vector<uint8_t> nal(p - 4, p + nal_size); // include 4-byte start code

                if (nal_unit_type == 32 || nal_unit_type == 33 || nal_unit_type == 34) {
                    out.push_back(nal);
                }

                p = p + nal_size;
            } else {
                ++p;
            }
        }

        return !out.empty();
    }

    std::vector<uint8_t> list_nal_unit_types_in_prefix(const uint8_t* data, size_t size, size_t max_prefix_length = 128) {
        std::vector<uint8_t> nal_types_found;

        size_t check_len = std::min(size, max_prefix_length);
        const uint8_t* p = data;
        const uint8_t* end = data + check_len;

        while (p + 5 < end) {
            if (p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x01) {
                uint8_t nal_unit_type = (p[4] >> 1) & 0x3F;
                nal_types_found.push_back(nal_unit_type);
                p += 4;
            } else {
                ++p;
            }
        }
        return nal_types_found;
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZMQGStreamerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}