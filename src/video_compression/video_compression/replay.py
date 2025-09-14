#!/usr/bin/env python3

import sys
import numpy as np
from PIL import Image
import zmq
import time

import struct

def read_log_frame(f):
    # Read binary header: 1 byte marker, 1 byte msg type, 4 bytes frame size, 4 bytes timing (ms)
    header = f.read(10)
    if not header or len(header) < 10:
        return None, None, None, None  # EOF
    marker, msg_type, frame_size, timestamp_ms = struct.unpack('>BBII', header)
    if marker != 0x01:
        raise ValueError(f"Invalid marker byte: {marker}")
    msg_type_chr = chr(msg_type)

    # Read payload
    data = f.read(frame_size)
    if len(data) != frame_size:
        raise ValueError("Unexpected EOF while reading binary data")

    return msg_type_chr, timestamp_ms, frame_size, data


def main():
    if len(sys.argv) != 2:
        print("Usage: python replay.py capture_n.log")
        sys.exit(1)

    log_file = sys.argv[1]

    # === ZMQ Setup ===
    ctx = zmq.Context()

    f = open(log_file, "rb")

    # Read the first block to get header info and set up queues accordingly
    msg_type_chr, timestamp_ms, framef_size, data = read_log_frame(f)
    if msg_type_chr != 'h':
        raise ValueError("First record is not an initial header (h)")
    header_info = data.decode('utf-8')
    print(f"Header: {header_info}")

    # header_info is f"{hostname};{actual_time};{topic_str}"
    parts = header_info.strip().split(";")
    if len(parts) != 3:
        raise ValueError("Header info format invalid")
    hostname, capture_time, topic_str = parts
    # topic_str is a colon-separated dict: "queue_path1!queue_name1,queue_path2!queue_name2,..."
    topics = {}
    publishers = {}
    topic_pairs = topic_str.strip().split(",")
    for pair in topic_pairs:
        if not pair:
            continue
        topic_path, topic_name = pair.split("!")
        topics[topic_path] = topic_name
        pub = ctx.socket(zmq.PUB)
        pub.bind(topic_path)
        publishers[topic_name] = pub   
    print(f"Topics found: {topics}")

    print("Streaming frames via ZMQ IPC...")

    try:
        player_start_time = time.time()
        while True:
            pos = f.tell()
            frame = read_log_frame(f)
            now = time.time()
            if frame[0] is None:
                f.seek(10 + framef_size)  # back to start but skip first header
                player_start_time = time.time()
                continue

            msg_type_chr, timestamp_ms, frame_size, data = frame
            pok = False
            for p, pub in publishers.items():
                if p[0] == msg_type_chr:
                    if msg_type_chr == 'c':
                        data = np.frombuffer(data, dtype=np.uint8)
                    if msg_type_chr == 'd':
                        data = np.frombuffer(data, dtype=np.uint16)
                    pub.send(data)
                    #print(f"Sent {p} message with length {len(data)} bytes")
                    pok = True
                    break
            if not pok:
                print(f"Unknown msg type {msg_type_chr}, skipping...")

            if timestamp_ms / 1000.0 > (now - player_start_time):
                time.sleep((timestamp_ms / 1000.0) - (now - player_start_time))

    except KeyboardInterrupt:
        f.close()

if __name__ == "__main__":
    main()
