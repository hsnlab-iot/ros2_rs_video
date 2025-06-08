import numpy as np
import cv2
from numba import njit

def encode(depth):
    result = depth16_to_12bit_piecewise(depth)
    result = encode_12bit_to_rgb_4bit_channels(result)
    return result

def decode(rgb_img):
    result = decode_rgb_to_12bit(rgb_img)
    result = depth12_to_16bit_piecewise(result)
    return result

def depth16_to_12bit_piecewise(depth):
    """
    Vectorized, fast conversion of 16-bit depth to 12-bit (0–4095) using 3 regions:
    - 0–4095       → 2x quantization (high precision)
    - 4096–16383   → 12x quantization
    - 16384–65535  → 48x quantization
    """
    depth = depth.astype(np.uint32)  # prevent overflow

    conditions = [
        depth <= 4095,
        (depth > 4095) & (depth <= 16383),
        depth > 16383
    ]

    choices = [
        depth >> 1,
        2048 + ((depth - 4096) // 12),
        3072 + ((depth - 16384) // 48)
    ]

    quantized = np.select(conditions, choices, default=4095).astype(np.uint16)
    return quantized

def depth12_to_16bit_piecewise(quantized):
    """
    Vectorized, fast conversion of 12-bit quantized values (0–4095) back to 16-bit depth.
    This is the approximate inverse of depth16_to_12bit_piecewise.
    """
    quantized = quantized.astype(np.uint16)

    conditions = [
        quantized < 2048,
        (quantized >= 2048) & (quantized < 3072),
        quantized >= 3072
    ]

    choices = [
        quantized.astype(np.uint32) << 1,
        4096 + ((quantized - 2048) * 12),
        16384 + ((quantized - 3072) * 48)
    ]

    depth = np.select(conditions, choices, default=65535).astype(np.uint16)
    return depth

@njit
def depth16_to_12bit_piecewise_numba(depth):
    h, w = depth.shape
    output = np.empty_like(depth, dtype=np.uint16)

    for y in range(h):
        for x in range(w):
            d = depth[y, x]
            if d <= 4095:
                output[y, x] = d >> 1
            elif d <= 16383:
                output[y, x] = 2048 + ((d - 4096) // 12)
            else:
                output[y, x] = 3072 + ((d - 16384) // 48)

    return output

@njit
def depth12_to_16bit_piecewise_numba(quantized):
    h, w = quantized.shape
    output = np.empty_like(quantized, dtype=np.uint16)

    for y in range(h):
        for x in range(w):
            q = quantized[y, x]
            if q < 2048:
                output[y, x] = q << 1
            elif q < 3072:
                output[y, x] = 4096 + ((q - 2048) * 12)
            else:
                output[y, x] = 16384 + ((q - 3072) * 48)

    return output

def encode_12bit_to_rgb_4bit_channels(depth12):
    """
    Encode 12-bit values (0–4095) to RGB image using 4 bits per channel (stored in high nibble).
    Each channel stores 4 bits of the depth:
      - R: bits 11–8
      - G: bits 7–4
      - B: bits 3–0
    """
    assert depth12.dtype == np.uint16
    assert depth12.max() <= 4095

    r = ((depth12 >> 8) & 0x0F).astype(np.uint8) << 4  # bits 11–8 → high nibble
    g = ((depth12 >> 4) & 0x0F).astype(np.uint8) << 4  # bits 7–4
    b = (depth12 & 0x0F).astype(np.uint8) << 4         # bits 3–0

    return np.stack((r, g, b), axis=-1)  # shape: (H, W, 3)

def decode_rgb_to_12bit(rgb_img):
    """
    Decode RGB image back to 12-bit uint16 depth.
    Assumes each channel uses only the high 4 bits (low nibble is 0).
    """
    r = (rgb_img[:, :, 0] >> 4).astype(np.uint16)
    g = (rgb_img[:, :, 1] >> 4).astype(np.uint16)
    b = (rgb_img[:, :, 2] >> 4).astype(np.uint16)

    return (r << 8) | (g << 4) | b

@njit
def encode_12bit_to_rgb_4bit_channels_numba(depth12):
    h, w = depth12.shape
    rgb = np.zeros((h, w, 3), dtype=np.uint8)

    for y in range(h):
        for x in range(w):
            val = depth12[y, x]
            r = ((val >> 8) & 0x0F) << 4  # bits 11–8 to high nibble
            g = ((val >> 4) & 0x0F) << 4  # bits 7–4
            b = (val & 0x0F) << 4         # bits 3–0

            rgb[y, x, 0] = r
            rgb[y, x, 1] = g
            rgb[y, x, 2] = b

    return rgb

@njit
def decode_rgb_to_12bit_numba(rgb):
    h, w, _ = rgb.shape
    depth12 = np.zeros((h, w), dtype=np.uint16)

    for y in range(h):
        for x in range(w):
            r = rgb[y, x, 0] >> 4
            g = rgb[y, x, 1] >> 4
            b = rgb[y, x, 2] >> 4
            val = (r << 8) | (g << 4) | b
            depth12[y, x] = val

    return depth12

def depth16_to_rgb_hue_value(depth16):
    """
    Encode 16-bit depth into HSV using:
    - Hue: high 8 bits (scaled to 0–360)
    - Value: low 8 bits (scaled to 0–1)
    Converts to RGB for saving/display.
    """
    assert depth16.dtype == np.uint16

    depth16 = depth16.astype(np.uint16)
    high = (depth16 >> 8).astype(np.uint8)  # 8 MSB
    low = (depth16 & 0xFF).astype(np.uint8) # 8 LSB

    hsv = np.zeros((*depth16.shape, 3), dtype=np.float32)
    hsv[..., 0] = high.astype(np.float32) * (360.0 / 256.0)  # Hue in degrees
    hsv[..., 1] = 1.0                                        # Full saturation
    hsv[..., 2] = low.astype(np.float32) / 255.0             # V in [0,1]

    rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    rgb8 = (rgb * 255).astype(np.uint8)
    return rgb8