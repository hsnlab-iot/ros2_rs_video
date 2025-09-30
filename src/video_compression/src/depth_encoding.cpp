#include "depth_encoding.h"
#include <cstring>
#include <algorithm>

namespace depth_encoding {

constexpr uint8_t LUT_4TO8[16] = {
     8,  24,  40,  56,
    72,  88, 104, 120,
   136, 152, 168, 184,
   200, 216, 232, 248
};

constexpr uint8_t LUT_2TO8GAP[4] = {
    28,   // symbol 0 -> q=3
    84,   // symbol 1 -> q=10
    172,  // symbol 2 -> q=21
    228   // symbol 3 -> q=28
};

constexpr uint8_t LUT_8TO2GAP[32] = {
    0, 0, 0, 0, 0, 0, 0,  // 0-6 -> symbol 0
    1, 1, 1, 1, 1, 1, 1,  // 7-13 -> symbol 1
    0xFF, 0xFF, 0xFF, 0xFF, // 14-17 -> invalid
    2, 2, 2, 2, 2, 2, 2,  // 18-24 -> symbol 2
    3, 3, 3, 3, 3, 3, 3   // 25-31 -> symbol 3
};

void encode_12bit_to_yuv422(uint16_t data, uint8_t *yuv)
{
    uint8_t y0_code = (data >> 8) & 0x0F;
    uint8_t y1_code = (data >> 4) & 0x0F;
    uint8_t u_code = (data >> 2) & 0x03;
    uint8_t v_code = (data) & 0x03;

    yuv[0] = LUT_4TO8[y0_code];
    yuv[1] = LUT_2TO8GAP[u_code];
    yuv[2] = LUT_4TO8[y1_code];
    yuv[3] = LUT_2TO8GAP[v_code];
}

void decode_12bit_from_yuv422(const uint8_t* yuv, uint16_t* data)
{
    uint8_t y0_code = yuv[0] >> 4; // equivalent to >> 2 >> 2
    uint8_t u_code = LUT_8TO2GAP[yuv[1] >> 3]; // equivalent to >> 5 >> 3
    uint8_t y1_code = yuv[2] >> 4;
    uint8_t v_code = LUT_8TO2GAP[yuv[3] >> 3];

    *data = (static_cast<uint16_t>(y0_code) << 8) |
            (static_cast<uint16_t>(y1_code) << 4) |
            (static_cast<uint16_t>(u_code) << 2) |
            static_cast<uint16_t>(v_code);
}

void encode_12bit_to_rgb(uint16_t data, uint8_t *rgb)
{
    uint8_t r4 = (data >> 8) & 0xF;
    uint8_t g4 = (data >> 4) & 0xF;
    uint8_t b4 = data & 0xF;
    rgb[0] = LUT_4TO8[r4];
    rgb[1] = LUT_4TO8[g4];
    rgb[2] = LUT_4TO8[b4];
}

void decode_12bit_from_rgb(const uint8_t* rgb, uint16_t* data)
{
    uint8_t r4 = rgb[0] >> 4; // equivalent to >> 2 >> 2
    uint8_t g4 = rgb[1] >> 4;
    uint8_t b4 = rgb[2] >> 4;
    *data = (static_cast<uint16_t>(r4) << 8) |
            (static_cast<uint16_t>(g4) << 4) |
            static_cast<uint16_t>(b4);
}


void depth_to_rgb(const uint16_t* ddata, uint8_t* depth_rgb, int width, int height)
{
    #pragma omp parallel for
    for (int i = 0; i < width * height; ++i) {
        encode_12bit_to_rgb(ddata[i], &depth_rgb[i * 3]);
    }
}

void rgb_to_depth(const uint8_t* depth_rgb, uint16_t* ddata, int width, int height)
{
    #pragma omp parallel for
    for (int i = 0; i < width * height; ++i) {
        decode_12bit_from_rgb(&depth_rgb[i * 3], &ddata[i]);
    }
}

void depth_to_yuv(const uint16_t* ddata, uint8_t* depth_yuv, int width, int height)
{
    #pragma omp parallel for
    for (int i = 0; i < width * height; ++i) {
        encode_12bit_to_yuv422(ddata[i], &depth_yuv[i * 4]);
    }
}

void yuv_to_depth(const uint8_t* depth_yuv, uint16_t* ddata, int width, int height)
{
    #pragma omp parallel for
    for (int i = 0; i < width * height; ++i) {
        decode_12bit_from_yuv422(&depth_yuv[i * 4], &ddata[i]);
    }
}

void depth_to_yuv_12(const uint16_t* ddata, uint16_t* depth_yuv_12, int width, int height)
{
    std::memcpy(depth_yuv_12, ddata, width * height * 2);
    std::memset(depth_yuv_12 + width * height * 2, width * height * 4, 0x00);
}

void yuv_12_to_depth(const uint16_t* depth_yuv_12, uint16_t* ddata, int width, int height)
{
    std::memcpy(ddata, depth_yuv_12, width * height * 2);
}

}