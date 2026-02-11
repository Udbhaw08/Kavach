import os
# Fix protobuf compatibility for older pygazebo
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import numpy as np
import cv2
import pygazebo
from pygazebo import connect
from pygazebo.msg.image_pb2 import Image

async def main():
    print("Connecting to Gazebo...")
    try:
        manager = await connect()
        print("Connected to Gazebo!")
    except Exception as e:
        print(f"Failed to connect to Gazebo: {e}")
        return

    topic_name = '/gazebo/long_range_test/vtol_downward_depth_camera/depth_camera/link/camera/image'
    
    print("\n[DEBUG] Listing available Gazebo topics (first 10):")
    pubs = manager.publications()
    for topic, msg_type in pubs[:10]:
        print(f" - {topic} ({msg_type})")
    
    if any(p[0] == topic_name for p in pubs):
        print(f"\n[SUCCESS] Topic '{topic_name}' found in publications!")
    else:
        print(f"\n[WARNING] Topic '{topic_name}' NOT found in publications!")
        print("Maybe check the exact topic name?")

    try:
        subscriber = manager.subscribe(
            topic_name,
            'gazebo.msgs.Image',
            lambda data: asyncio.create_task(on_image(data))
        )
        print(f"Subscribed to {topic_name}")
    except Exception as e:
        print(f"Failed to subscribe: {e}")
        return

    # Keep the script running
    while True:
        await asyncio.sleep(1)

async def on_image(data):
    if not hasattr(on_image, "counter"): on_image.counter = 0
    on_image.counter += 1
    if on_image.counter % 30 == 0:
        print(".", end="", flush=True)

    try:
        img = Image()
        img.ParseFromString(data)

        width = img.width
        height = img.height
        
        # Convert raw data to numpy array
        # Gazebo usually sends RGB8
        raw_data = np.frombuffer(img.data, dtype=np.uint8)
        
        # Check if the data size matches expected size
        if raw_data.size != width * height * 3:
            # Try determining format from pixel_format field if needed, but for now assume RGB8
            print(f"Warning: Data size {raw_data.size} does not match {width}x{height}x3")
            return

        frame = raw_data.reshape((height, width, 3))
        
        # Convert RGB to BGR for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # ---- SIMPLE PoC DETECTION ----
        # Convert to HSV for color masking
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Tune these values for the tank's color (green/olive)
        # Olive green is roughly H: 40-80, S: 30-100, V: 30-100
        # Let's start with a broad green range
        lower_green = np.array([30, 40, 40])
        upper_green = np.array([80, 255, 255])

        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Morphological operations to remove noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        for c in contours:
            area = cv2.contourArea(c)
            if area > 800: # Threshold as requested
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                cv2.putText(frame, f"TANK ({int(area)})", (x,y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                detected = True

        cv2.imshow("Gazebo Camera PoC", frame)
        cv2.imshow("Mask", mask) # Show mask for debugging
        
        if cv2.waitKey(1) == 27: # Esc to exit
            exit()
            
    except Exception as e:
        print(f"Error processing image: {e}")

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
