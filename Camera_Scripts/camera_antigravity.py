import gz.transport13 as gz
# from gz.msgs10.image_pb2 import Image # Not needed for raw subscription
import cv2
import numpy as np
import gzip

# callback receives raw serialized ImageStamped message
# subscribe_raw passes (msg_bytes, msg_info)
def on_image(msg_bytes, _msg_info=None):
    try:
        # The message is gzipped in Gazebo Classic!
        msg_str = gzip.decompress(msg_bytes)

        # Extract header length (first 4 bytes)
        header_size = int.from_bytes(msg_str[:4], byteorder="little")

        # Extract protobuf content
        proto = msg_str[4:4+header_size]

        # The rest is raw RGB data
        img_data = msg_str[4+header_size:]

        # Parse protobuf manually for width/height (small hack)
        # Gazebo packs width/height as varints with tags 2 & 3
        width = int.from_bytes(proto.split(b'\x10')[1].split(b'\x18')[0], 'little')
        height = int.from_bytes(proto.split(b'\x18')[1].split(b'\x22')[0], 'little')

        # Decode RGB pixels
        img = np.frombuffer(img_data, dtype=np.uint8).reshape(height, width, 3)

        cv2.imshow("Gazebo Camera Feed", img)
        cv2.waitKey(1)

    except Exception as e:
        print("\nError decoding image:", e)

print("Listening to Gazebo camera...")

node = gz.Node()

topic = "/gazebo/default/iris_demo/iris_demo/front_rgb_camera_link/front_rgb_camera/image"

# Using subscribe_raw to bypass message type class requirement
# Topic type name is "gz.msgs.Image"
ok = node.subscribe_raw(topic, on_image, "gz.msgs.Image", gz.SubscribeOptions())

if not ok:
    print("❌ Subscription failed. Topic unavailable.")
else:
    print("✅ Subscribed! Waiting for frames...")

while True:
    pass
