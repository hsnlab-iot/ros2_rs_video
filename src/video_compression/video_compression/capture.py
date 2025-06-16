import zmq
import os
import time

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

# Set up sockets and files
for endpoint, name in topics.items():
    sock = context.socket(zmq.SUB)
    sock.connect(endpoint)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    poller.register(sock, zmq.POLLIN)
    sockets[sock] = name
    files[name] = open(f"{name}_data.log", "ab")  # append in binary mode

print("Listening to topics...")
print("\n".join(f"{ep} → {fn}" for ep, fn in topics.items()))

try:
    while True:
        events = dict(poller.poll(timeout=1000))  # ms

        for sock in sockets:
            if sock in events and events[sock] == zmq.POLLIN:
                msg = sock.recv()
                timestamp_ms = int(time.time() * 1000)
                msg_size = len(msg)

                # Write header: timestamp_ms<TAB>size\n
                header = f"{timestamp_ms},{msg_size}\n".encode("utf-8")

                file = files[sockets[sock]]
                file.write(header)
                file.write(msg)
                file.write(b"\n===\n")  # optional delimiter
                file.flush()
                print(".")

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    for f in files.values():
        f.close()
    for s in sockets:
        s.close()
    context.term()
