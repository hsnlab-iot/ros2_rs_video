#pragma once
#include <cstdint>
#include <vector>

namespace depth_encoding {

void depth_to_rgb(const uint16_t* ddata, uint8_t* depth_rgb, int width, int height);
void rgb_to_depth(const uint8_t* depth_rgb, uint16_t* ddata, int width, int height);
void depth_to_yuv(const uint16_t* ddata, uint8_t* depth_yuv, int width, int height);
void yuv_to_depth(const uint8_t* depth_yuv, uint16_t* ddata, int width, int height);
void depth_to_yuv_12(const uint16_t* data, uint16_t* depth_yuv_12, int width, int height);

}