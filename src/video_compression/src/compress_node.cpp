#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <zmq.h>
#include <thread>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <cstring>

#include "depth_encoding.h"
#include "image_marking.h"


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
        compression_ = this->declare_parameter<std::string>("compression", "h265");
        custom_pipeline_ = this->declare_parameter<std::string>("pipeline", "");
        custom_codec_ = this->declare_parameter<std::string>("codec", "");
        embed_ = this->declare_parameter<std::string>("embed", "none");

        if (mode_ != "color" && mode_ != "depth_rgb" && mode_ != "depth_yuv" && mode_ != "depth_yuv_12") {
            RCLCPP_ERROR(this->get_logger(), "Invalid mode specified: %s. Supported modes are: color, depth_rgb, depth_yuv, depth_yuv_12", mode_.c_str());
            throw std::runtime_error("Invalid mode specified.");
        }
        if (custom_codec_.empty() && custom_pipeline_.empty()) {
            if (compression_ != "h264" && compression_ != "h265" && compression_ != "rpi_h264") {
                RCLCPP_ERROR(this->get_logger(), "Invalid compression specified: %s. Supported compressions are: h264, rpi_h264, h265", compression_.c_str());
                throw std::runtime_error("Invalid compression specified.");
            }
        }
        static const std::vector<std::string> embed_mode_names = { "none", "sequence", "time", "mix" };
        auto it = std::find(embed_mode_names.begin(), embed_mode_names.end(), embed_);
        if (it != embed_mode_names.end()) {
            embed_mode = static_cast<int>(std::distance(embed_mode_names.begin(), it));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid embed mode specified: %s. Supported modes are: none, sequence, time, mix", embed_.c_str());
            throw std::runtime_error("Invalid embed mode specified.");
        }

        depth_mode = false;
        xwidth_ = width_;

        if (mode_ == "color") {
            rawformat_ = "RGB";
            encformat_ = "I420";
        } else if (mode_ == "depth_rgb") {
            rawformat_ = "RGB";
            encformat_ = "I420";
            depth_mode = true;
            depth_rgb = new uint8_t[width_ * height_ * 3];
        } else if (mode_ == "depth_yuv") {
            rawformat_ = "YUY2";
            encformat_ = "YUY2";
            depth_mode = true;
            depth_yuv = new uint8_t[width_ * height_ * 4];
            xwidth_ = width_ * 2; // YUY2 is doubles width
        } else if (mode_ == "depth_yuv_12") {
            rawformat_ = "Y444_12LE";
            encformat_ = "Y444_12LE";
            depth_mode = true;
            depth_yuv_12 = new uint16_t[width_ * height_ * 3];
        }

        gst_init(nullptr, nullptr);

        std::ostringstream pipeline_ss;
        if (!custom_pipeline_.empty()) {
            pipeline_ss << custom_pipeline_;
        } else {
            pipeline_ss
                << "appsrc name=src is-live=true format=time do-timestamp=true ! videoconvert ! "
                << "queue max-size-buffers=1 leaky=downstream ! "
                << "video/x-raw,format=" << encformat_ << ",width=" << xwidth_ << ",height=" << height_ << ",framerate=" << rate_ << "/1 ! ";

            if (!custom_codec_.empty()) {
                pipeline_ss << custom_codec_ << " ! ";
            } else {
                if (compression_ == "h265") {
                    pipeline_ss
                        << "x265enc tune=zerolatency bitrate=" << bitrate_ << " speed-preset=ultrafast key-int-max=15 ! "
                        << "video/x-h265,stream-format=byte-stream,alignment=au ! ";
                } else if (compression_ == "h264") {
                    pipeline_ss
                        << "x264enc tune=zerolatency bitrate=" << bitrate_ << " speed-preset=ultrafast b-adapt=false key-int-max=15 sliced-threads=true byte-stream=true ! "
                        << "video/x-h264,profile=main,stream-format=byte-stream,alignment=nal ! ";
                } else if (compression_ == "rpi_h264") {
                    pipeline_ss
                        << "v4l2h264enc output-io-mode=mmap capture-io-mode=mmap extra-controls=\"controls,video_bitrate=" << bitrate_ * 1000
                        << ",video_bitrate_mode=1,h264_i_frame_period=15,repeat_sequence_header=1\" ! video/x-h264,level=(string)4 ! "
                        << "queue max-size-buffers=16 leaky=downstream ! "
                        << "h264parse config-interval=1 ! video/x-h264,stream-format=byte-stream,alignment=nal ! ";
                }
            }

            pipeline_ss
                << "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true";
        }

        pipeline_ = gst_parse_launch(pipeline_ss.str().c_str(), nullptr);
        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        
        // Get the bus from the pipeline for message handling
        bus_ = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
        
        if (!appsrc_ || !appsink_) {
            RCLCPP_ERROR(this->get_logger(), "GStreamer pipeline must contain named 'appsrc' and 'appsink' elements.");
            throw std::runtime_error("Invalid GStreamer pipeline.");
        }

        GstCaps* caps = gst_caps_new_simple(
            "video/x-raw",
            "format", G_TYPE_STRING, rawformat_.c_str(),
            "width", G_TYPE_INT, xwidth_,
            "height", G_TYPE_INT, height_,
            "framerate", GST_TYPE_FRACTION, rate_, 1,
            NULL
        );
        g_object_set(appsrc_, "caps", caps, NULL);
        gst_caps_unref(caps);

        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline: %s", pipeline_ss.str().c_str());
        GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline_dump");

        g_signal_connect(appsink_, "new-sample", G_CALLBACK(&ZMQGStreamerPublisher::on_new_sample_static), this);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("video/h264", 3);

        // Set up timer to check bus messages periodically
        bus_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // Check every 50ms
            std::bind(&ZMQGStreamerPublisher::handle_bus_messages, this)
        );

        zmq_thread_ = std::thread(&ZMQGStreamerPublisher::zmq_listener, this);
    }

    ~ZMQGStreamerPublisher() override {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        if (bus_) {
            gst_object_unref(bus_);
        }
        gst_object_unref(appsrc_);
        gst_object_unref(appsink_);
        gst_object_unref(pipeline_);
        if (zmq_thread_.joinable()) {
            zmq_thread_.join();
        }
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

private:
    std::string zmq_url_, mode_, encformat_, rawformat_, compression_, custom_pipeline_, custom_codec_, embed_;
    int width_, xwidth_, height_, rate_, bitrate_;
    bool depth_mode;
    enum EmbedMode { EMBED_NONE = 0, EMBED_SEQUENCE, EMBED_TIME, EMBED_MIX };
    int embed_mode = 0; // 0: none, 1: sequence, 2: time, 3: mix
    GstElement *pipeline_{nullptr}, *appsrc_{nullptr}, *appsink_{nullptr};
    GstBus *bus_{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr bus_timer_;
    std::thread zmq_thread_;
    size_t zmq_msg_count_ = 0; // ZMQ message counter
    size_t ros_msg_count = 0; // ROS message counter

    uint8_t *depth_rgb = nullptr;
    uint8_t *depth_yuv = nullptr;
    uint16_t *depth_yuv_12 = nullptr;

    std::vector<std::vector<uint8_t>> parameter_sets;   // VPS, SPS, PPS parameter sets
    bool extracted_parameter_sets = false;

    void handle_bus_messages() {
        while (true) {
            GstMessage* msg = gst_bus_pop(bus_);
            if (!msg) break;

            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR: {
                    GError* err;
                    gchar* debug_info;
                    gst_message_parse_error(msg, &err, &debug_info);
                    RCLCPP_ERROR(this->get_logger(), "GStreamer Error: %s", err->message);
                    g_error_free(err);
                    g_free(debug_info);
                    break;
                }
                case GST_MESSAGE_EOS:
                    RCLCPP_INFO(this->get_logger(), "GStreamer End-Of-Stream reached.");
                    break;
                default:
                    break;
            }
            gst_message_unref(msg);
        }
    }

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
                if (depth_mode && (size != width_ * height_ * 2)) {
                    RCLCPP_ERROR(this->get_logger(), "Received depth data size mismatch: expected %d, got %ld", width_ * height_ * 2, size);
                    continue;
                }
                if (!depth_mode && (size != width_ * height_ * 3)) {
                    RCLCPP_ERROR(this->get_logger(), "Received color data size mismatch: expected %d, got %ld", width_ * height_ * 3, size);
                    continue;
                }
                void* data = zmq_msg_data(&msg);

                ++zmq_msg_count_;
                if (zmq_msg_count_ % 100 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Received %zu ZMQ messages. Actual size: %ld", zmq_msg_count_, size);
                }

                // Embed frmae count code into the image
                if (embed_mode != EMBED_NONE) {
                    uint32_t code = 0;
                    if (embed_mode == EMBED_SEQUENCE) {
                        code = static_cast<uint32_t>(ros_msg_count & 0xFFFFFFFF);
                    } else if (embed_mode == EMBED_TIME) {
                        code = image_marking::time_to_code32(this->now());
                    } else if (embed_mode == EMBED_MIX) {
                        code = static_cast<uint32_t>(((ros_msg_count & 0xFFFF) << 16) |
                             (image_marking::time_to_code16(this->now()) & 0xFFFF));
                    }
                    if (!depth_mode) {
                        image_marking::embed_code(static_cast<uint8_t*>(data), width_, 3, code);
                    } else {
                        image_marking::embed_code(static_cast<uint8_t*>(data), width_, 2, code);
                    }
                }

                if (mode_ == "depth_rgb") {
                    uint16_t* ddata = static_cast<uint16_t*>(zmq_msg_data(&msg));
                    depth_encoding::depth_to_rgb(ddata, depth_rgb, width_, height_);
                    data = static_cast<void*>(depth_rgb);
                    size = width_ * height_ * 3;
                }
                else if (mode_ == "depth_yuv") {
                    uint16_t* ddata = static_cast<uint16_t*>(zmq_msg_data(&msg));
                    depth_encoding::depth_to_yuv(ddata, depth_yuv, width_, height_);
                    data = static_cast<void*>(depth_yuv);
                    size = width_ * height_ * 4;
                }
                else if (mode_ == "depth_yuv_12") {
                    uint16_t* ddata = static_cast<uint16_t*>(zmq_msg_data(&msg));
                    depth_encoding::depth_to_yuv_12(ddata, depth_yuv_12, width_, height_);
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

        if (compression_ == "h265" && !extracted_parameter_sets) {
            RCLCPP_INFO(this->get_logger(), "Extracting VPS/SPS/PPS from H.265 stream");
            // Extract VPS/SPS/PPS once
            if (!extracted_parameter_sets) {
                extract_vps_sps_pps(map.data, map.size, parameter_sets);
                if (!parameter_sets.empty()) {
                    extracted_parameter_sets = true;
                }
            }
        }

        std::vector<uint8_t> nal_types;
        auto has_vps_sps_pps = false;
        auto keyframe = false;

        if (compression_ == "h265") {
            nal_types = list_nal_unit_types_in_prefix(map.data, map.size);
            has_vps_sps_pps = nal_types.end() != std::find(nal_types.begin(), nal_types.end(), 32) ||
                            nal_types.end() != std::find(nal_types.begin(), nal_types.end(), 33) ||
                            nal_types.end() != std::find(nal_types.begin(), nal_types.end(), 34);
            keyframe = nal_types.end() != std::find(nal_types.begin(), nal_types.end(), 19) ||
                            nal_types.end() != std::find(nal_types.begin(), nal_types.end(), 20) ||
                            nal_types.end() != std::find(nal_types.begin(), nal_types.end(), 21);
        }
                            
        /* DEBUG only
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
        msg.format = compression_ == "h265" ? "hevc" : "h264";
    
        std::vector<uint8_t> new_data;
        if (compression_ == "h265" && keyframe && extracted_parameter_sets && !has_vps_sps_pps) {
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

        ++ros_msg_count;
        if (ros_msg_count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Published %zu ROS messages", ros_msg_count);
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
