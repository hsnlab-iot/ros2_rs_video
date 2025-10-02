#pragma once
#include <cstdint>
#include <vector>

// Embed a 32-bit code visually into the image (simple LSB watermarking)
// img: pointer to image data (uint8_t*), size: width*height*channels
// width, height: image dimensions
// channels: number of channels (e.g., 3 for RGB)
// code: 32-bit code to embed
void embed_code(uint8_t* img, int width, int height, int channels, uint32_t code);

// Try to detect the 32-bit code from the image (reverse of embed_code)
uint32_t detect_code(const uint8_t* img, int width, int height, int channels);