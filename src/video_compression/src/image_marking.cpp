#include "image_marking.h"

#define CODE_ROWS 8

namespace image_marking {

// Overwriute the first rows with the code bits
void embed_code(uint8_t* img, int width, int height, int channels, uint32_t code) {
    int pixels = width * CODE_ROWS;
    int pixel_per_bit = width / 32;
    for (int r = 0; r < CODE_ROWS; ++r) {
        for (int b = 0; b < 32; ++b) {
            int idx = (r * width + b * pixel_per_bit) * channels;
            if ((code >> b) & 1)
                for (int i = 0; i < pixel_per_bit * channels; ++i) img[idx + i] = 255;
            else
                for (int i = 0; i < pixel_per_bit * channels; ++i) img[idx + i] = 0;
        }
    }
}

uint32_t detect_code(const uint8_t* img, int width, int height, int channels, float &confidence) {
    uint32_t code = 0;
    int pixel_per_bit = width / 32;
    std::vector<float> confidance_b;
    confidance_b.resize(32);
    for (int b = 0; b < 32; ++b) {
        int black_pixels = 0;
        int white_pixels = 0;
        for (int r = 0; r < CODE_ROWS; ++r) {
            int idc = (r * width + b * pixel_per_bit) * channels;
            for (int i = 0; i < pixel_per_bit * channels; ++i) {
                if (img[idc + i] < 128) black_pixels++;
                else white_pixels++;
            }
        }
        if (black_pixels < white_pixels) code |= (1 << b);
        confidance_b[b] = std::max(black_pixels, white_pixels) / float(black_pixels + white_pixels);
    }
    confidence = 1;
    for (int b = 0; b < 32; ++b) confidence *= confidance_b[b];

    return code;
}

uint16_t time_to_code16(rclcpp::Time time) {
    uint64_t sec = static_cast<uint64_t>(time.seconds());
    uint16_t sec_4 = static_cast<uint16_t>(sec & 0xF);
    uint16_t msec = static_cast<uint16_t>(((time.nanoseconds() / 1000000) % 1000));

    return (sec_4 << 12) | msec;
}

uint32_t time_to_code32(rclcpp::Time time) {
    uint64_t sec = static_cast<uint64_t>(time.seconds());
    uint16_t sec_16 = static_cast<uint16_t>(sec & 0xFFFF);
    uint32_t msec = static_cast<uint32_t>(((time.nanoseconds() / 1000000) % 1000));

    return (static_cast<uint32_t>(sec_16) << 16) | msec;
}

rclcpp::Time code16_to_time(uint16_t code, rclcpp::Time base_time) {
    uint8_t c_sec_4 = (code >> 12) & 0xF;
    uint16_t c_msec = code & 0xFFF;

    uint64_t sec = static_cast<uint64_t>(base_time.seconds());
    uint8_t sec_4 = static_cast<uint8_t>(sec & 0xF);

    uint8_t mod = 0;
    if (c_sec_4 > sec_4 && ((c_sec_4 - sec_4) > 8)) {
        // we have wrapped around the 4-bit seconds
        mod = -1;
    } else if (c_sec_4 < sec_4 && ((sec_4 - c_sec_4) > 8)) {
        // we have wrapped around the 4-bit seconds
        mod = 1;
    }
    sec += mod * 16;
    sec = (sec & -0xF) | c_sec_4;

    return rclcpp::Time(sec, static_cast<uint32_t>(c_msec) * 1000000);
}

rclcpp::Time code32_to_time(uint32_t code, rclcpp::Time base_time) {
    uint16_t c_sec_16 = (code >> 16) & 0xFFFF;
    uint16_t c_msec = code & 0xFFFF;

    uint64_t sec = static_cast<uint64_t>(base_time.seconds());
    uint16_t sec_16 = static_cast<uint16_t>(sec & 0xFFFF);

    int32_t mod = 0;
    if (c_sec_16 > sec_16 && ((c_sec_16 - sec_16) > 32768)) {
        // we have wrapped around the 16-bit seconds
        mod = -1;
    } else if (c_sec_16 < sec_16 && ((sec_16 - c_sec_16) > 32768)) {
        // we have wrapped around the 16-bit seconds
        mod = 1;
    }

    sec += mod * 65536;
    sec = (sec & -0xFFFF) | c_sec_16;

    return rclcpp::Time(sec, static_cast<uint32_t>(c_msec) * 1000000);
}

} // namespace image_marking