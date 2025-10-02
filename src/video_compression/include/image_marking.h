#pragma once
#include <cstdint>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace image_marking {

// Embed a 32-bit code visually into the image (simple LSB watermarking)
// img: pointer to image data (uint8_t*), size: width*height*channels
// width, height: image dimensions
// channels: number of channels (e.g., 3 for RGB)
// code: 32-bit code to embed
void embed_code(uint8_t* img, int width, int channels, uint32_t code);

// Try to detect the 32-bit code from the image (reverse of embed_code)
uint32_t detect_code(const uint8_t* img, int width, int channels, float& confidence);
uint32_t detect_code_word(const uint16_t* img, int width, float& confidence);

rclcpp::Time code16_to_time(uint16_t code, rclcpp::Time base_time);
rclcpp::Time code32_to_time(uint32_t code, rclcpp::Time base_time);
uint16_t time_to_code16(rclcpp::Time time);
uint32_t time_to_code32(rclcpp::Time time);

}