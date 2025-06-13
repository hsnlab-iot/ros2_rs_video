#!/usr/bin/env python3

import sys
import numpy as np
from PIL import Image
import zmq
import time

def read_log_frame(f):
    # Read ASCII header: <epoch>,<length>\n
    epoch_bytes = b""
    while True:
        c = f.read(1)
        if not c:
            return None, None, None  # EOF
        if c == b',':
            break
        epoch_bytes += c
    epoch = int(epoch_bytes.decode())
    print(f"Epoch: {epoch}")

    length_bytes = b""
    while True:
        c = f.read(1)
        if not c:
            raise ValueError("Unexpected EOF while reading length")
        if c == b'\n':
            break
        length_bytes += c
    length = int(length_bytes.decode())
    print(f"Length: {length}")

    # Read binary data
    data = f.read(length)
    if len(data) != length:
        raise ValueError("Unexpected EOF while reading binary data")

    # Read separator "\n===\n"
    sep = f.read(5)
    if sep != b"\n===\n":
        raise ValueError("Expected separator '\\n===\\n' after binary data")

    return epoch, length, data

def main():
    if len(sys.argv) != 3:
        print("Usage: python log2ply.py color_log.bin depth_log.bin")
        sys.exit(1)

    color_log = sys.argv[1]
    depth_log = sys.argv[2]
    FPS = 30.0  # Frames per second

    # === ZMQ Setup ===
    ctx = zmq.Context()

    color_pub = ctx.socket(zmq.PUB)
    color_pub.bind("ipc:///tmp/color_image")

    depth_pub = ctx.socket(zmq.PUB)
    depth_pub.bind("ipc:///tmp/depth_image")

    print("Streaming frames via ZMQ IPC...")

    with open(color_log, "rb") as cf, open(depth_log, "rb") as df:
        while True:
            color_frame = read_log_frame(cf)
            depth_frame = read_log_frame(df)

            # If either file is at EOF, restart both
            if color_frame[0] is None or depth_frame[0] is None:
                cf.seek(0)
                df.seek(0)
                continue

            # Convert to numpy
            color_image = np.frombuffer(color_frame[2], dtype=np.uint8)
            depth_image = np.frombuffer(depth_frame[2], dtype=np.uint16)

            # Send messages
            color_pub.send(color_image)
            print (f"Sent color image with shape {color_image.shape} and length {len(color_image)} bytes")
            depth_pub.send(depth_image)
            print (f"Sent depth image with shape {depth_image.shape} and length {len(depth_image)} bytes")

            time.sleep(1.0 / FPS)


if __name__ == "__main__":
    main()