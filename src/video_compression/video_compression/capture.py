import zmq
import os
import time
import struct
import socket
from glob import glob

# === CONFIG ===
topics = {
    "ipc:///tmp/color_image": "color",
    "ipc:///tmp/depth_image": "depth",
}

# === PREP ===
context = zmq.Context()
sockets = {}
poller = zmq.Poller()
files = {}


# Find next sequence number for log files
def get_next_seq(name):
    pattern = f"{name}_data_*.log"
    existing = glob(pattern)
    nums = [int(f.split('_data_')[1].split('.log')[0]) for f in existing if '_data_' in f and f.endswith('.log') and f.split('_data_')[1].split('.log')[0].isdigit()]
    return max(nums, default=0) + 1

seq = get_next_seq('capture')

for endpoint, name in topics.items():
    sock = context.socket(zmq.SUB)
    sock.connect(endpoint)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    poller.register(sock, zmq.POLLIN)
    sockets[sock] = name

file = open(f"capture_{seq}.log", "wb")  # always new file

# Write initial header (type h)
hostname = socket.gethostname()
actual_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
topic_str = ','.join(f"{k}!{v}" for k, v in topics.items())
first_payload = f"{hostname};{actual_time};{topic_str}"
first_bytes = first_payload.encode('utf-8')
marker = 0x01
msg_type = b'h'
header = struct.pack('>BBII', marker, ord(msg_type), len(first_bytes), 0)

file.write(header)
file.write(first_bytes)
file.flush()

print("Listening to topics...")
print("\n".join(f"{ep} → {fn}" for ep, fn in topics.items()))

try:
    start_time = time.time()
    # Store (timestamp, topic) for last 30s
    frame_history = {"color": [], "depth": []}
    last_status_time = start_time

    while True:
        events = dict(poller.poll(timeout=1000))  # ms

        for sock in sockets:
            if sock in events and events[sock] == zmq.POLLIN:
                msg = sock.recv()
                now = time.time()
                timestamp_ms = int((now - start_time) * 1000)
                msg_size = len(msg)

                # Build binary header: 1 byte marker, 1 byte msg type, 4 bytes frame size, 4 bytes timing (ms)
                marker = 0x01
                topic_name = sockets[sock]
                msg_type = topic_name[0].encode('utf-8')
                header = struct.pack('>BBII', marker, ord(msg_type), msg_size, timestamp_ms)

                file.write(header)
                file.write(msg)
                file.flush()

                # Add to history
                frame_history[topic_name].append(now)
                # Remove frames older than 30s
                cutoff = now - 30
                frame_history[topic_name] = [t for t in frame_history[topic_name] if t >= cutoff]

        # Print status every 10 seconds
        current_time = time.time()
        if current_time - last_status_time >= 10:
            color_fps = len(frame_history["color"]) / 30.0
            depth_fps = len(frame_history["depth"]) / 30.0
            print(f"[STATUS] Last 30s: Color FPS={color_fps:.2f}, Depth FPS={depth_fps:.2f}")
            last_status_time = current_time

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    file.close()
    for s in sockets:
        s.close()
    context.term()
