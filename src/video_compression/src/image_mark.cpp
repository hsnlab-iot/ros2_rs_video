#include "image_mark.h"

#define CODE_ROWS 8

// Overwriute the first rows with the code bits
void embed_code(uint8_t* img, int width, int height, int channels, uint32_t code) {
    int pixels = width * CODE_ROWS;
    int pixel_per_bit = width / 32;
    for (int r = 0; r < CODE_ROWS; ++r) {
        for (int b = 0; b < 32; ++b) {
            int idx = (r * width + b * pixel_per_bit) * channels;
            if ((code >> b) & 1)
                for (int i = 0; i < pixel_per_bit * channels; ++i) img[idx + i] = 0;
            else
                for (int i = 0; i < pixel_per_bit * channels; ++i) img[idx + i] = 255;
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