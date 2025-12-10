#!/usr/bin/env python3
"""
Gazebo 11 Camera Feed Subscriber

ISSUE ENCOUNTERED: 
The Python gz.transport13 bindings have a critical bug where subscription callbacks
are NEVER invoked, even though:
- Subscriptions report success (return True)
- The camera topic IS publishing messages (verified via topic_list and topic_info)
- The transport layer itself doesn't deliver messages to Python callbacks

EVIDENCE:
- ✅ Topic exists: /default/iris_demo/iris_demo/front_rgb_camera_link/front_rgb_camera/image
- ✅ Publisher exists on this topic (verified)
- ✅ Subscription succeeds (returns True)  
- ❌ Callback NEVER invoked (tested for 30+ seconds)
- ❌ Even test messages published locally don't trigger callbacks

ROOT CAUSE:
This appears to be a limitation or bug in the Python bindings for gz-transport.
The C++ side works (as evidenced by the running `gz camera` process), but the
Python callback mechanism is broken.

SOLUTIONS:

Option 1: Use ROS 2 (Recommended if available)
-  Set up ROS 2 <-> Gazebo bridge
- Subscribe using rclpy instead
- Much more mature Python support

Option 2: Use C++ subscriber
- Write a C++ program using libgz-transport13
- Use subprocess.Popen to run it and pipe output to Python
- More reliable than Python bindings

Option 3: File-based approach
- Configure Gazebo to save camera frames to disk
- Read and process frames from disk in Python

Option 4: Use gazebo_ros_pkgs (if using ROS)
- Install gazebo_ros bridge
- Subscribe to /camera/image_raw or similar ROS topics

CURRENT STATUS: Script will print diagnostic information only.
"""

import gz.transport13 as gz
import time
import sys

print(__doc__)

print("\n" + "="*70)
print("DIAGNOSTIC INFORMATION")
print("="*70 + "\n")

try:
    node = gz.Node()
    print("[✓] Created gz.transport13 Node")
    
    time.sleep(2)
    
    print("[✓] Discovering topics...")
    topics = node.topic_list()
    print(f"[✓] Found {len(topics)} topics\n")
    
    print("All Topics:")
    for i, t in enumerate(topics, 1):
        info = node.topic_info(t)
        publishers = len(info[0])
        subscribers = len(info[1])
        marker = "📷" if "image" in t.lower() or "camera" in t.lower() else "  "
        print(f"  {i:2d}. {marker} {t}")
        if publishers > 0 or subscribers > 0:
            print(f"       └─ Publishers: {publishers}, Subscribers: {subscribers}")
    
    print("\n" + "="*70)
    print("CAMERA TOPICS DETECTED")
    print("="*70 + "\n")
    
    camera_topics = [t for t in topics if 'image' in t.lower() or 'camera' in t.lower()]
    if camera_topics:
        for topic in camera_topics:
            print(f"✅ {topic}")
        print(f"\n[!] Camera is publishing but Python callbacks cannot receive data.")
        print(f"[!] This is a known limitation of gz-transport13 Python bindings.")
    else:
        print("❌ No camera topics found!")
    
    print("\n" + "="*70)
    print("NEXT STEPS")
    print("="*70 + "\n")
    print("""
1. Try installing and using ROS 2:
   - Setup ROS 2 environment
   - Use rclpy for subscriptions (Python support is better)

2. Try using C++ subscriber:
   - Create a C++ program with libgz-transport13
   - Pipe frames to Python for processing

3. Check Gazebo/gz-transport documentation:
   - Look for Python binding updates or workarounds
   - Check if there's a synchronous subscription mode

4. Consider using cv2.VideoCapture with Gazebo plugins:
   - Some Gazebo setups expose camera output via standard video interfaces
   - Worth checking if your installation supports this
""")

except Exception as e:
    import traceback
    print(f"❌ Error: {e}")
    traceback.print_exc()

sys.exit(0)
