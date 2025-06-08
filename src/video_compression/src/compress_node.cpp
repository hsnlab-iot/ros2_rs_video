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

        if (mode_ != "color" && mode_ != "depth") {
            RCLCPP_ERROR(this->get_logger(), "Invalid mode specified. Use 'color' or 'depth'.");
            throw std::runtime_error("Invalid mode specified.");
        }

        gst_init(nullptr, nullptr);

        std::ostringstream pipeline_ss;
        pipeline_ss
            << "appsrc name=src ! "
            << "videoconvert ! "
            << "video/x-raw,format=I420,width=" << width_ << ",height=" << height_ << ",framerate=" << rate_ << "/1 ! "
            << "x264enc tune=zerolatency bitrate=" << bitrate_ << " speed-preset=ultrafast b-adapt=false key-int-max=3 sliced-threads=true byte-stream=true ! "
            //<< "video/x-h264,profile=main,stream-format=byte-stream,alignment=nal ! "
            << "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true";

        pipeline_ = gst_parse_launch(pipeline_ss.str().c_str(), nullptr);
        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");

        
        if (!appsrc_ || !appsink_) {
            RCLCPP_ERROR(this->get_logger(), "GStreamer pipeline must contain named 'appsrc' and 'appsink' elements.");
            throw std::runtime_error("Invalid GStreamer pipeline.");
        }

        // Set properties for appsrc
        GstCaps *caps = gst_caps_new_simple(
            "video/x-raw",
            "format", G_TYPE_STRING, "RGB",
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
    std::string zmq_url_, mode_;
    int width_, height_, rate_, bitrate_;
    GstElement *pipeline_{nullptr}, *appsrc_{nullptr}, *appsink_{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    std::thread zmq_thread_;
    size_t zmq_msg_count_ = 0; // ZMQ message counter
    size_t ros_msg_count_ = 0; // ROS message counter

    void zmq_listener() {
        RCLCPP_WARN(this->get_logger(), "Listening for ZMQ messages on %s", zmq_url_.c_str());
        void* context = zmq_ctx_new();
        void* socket = zmq_socket(context, ZMQ_SUB);
        zmq_connect(socket, zmq_url_.c_str());
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

        auto msg = sensor_msgs::msg::CompressedImage();
        msg.header.stamp = this->now();
        msg.format = "h264";
        msg.data.assign(map.data, map.data + map.size);

        publisher_->publish(msg);

        ++ros_msg_count_;
        if (ros_msg_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Published %zu ROS messages", ros_msg_count_);
        }

        gst_buffer_unmap(buf, &map);
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZMQGStreamerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}