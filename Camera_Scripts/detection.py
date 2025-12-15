import cv2
import numpy as np
import time

# Try different video sources
# Option 1: GStreamer pipeline
gst_pipeline = (
    "udpsrc port=5600 ! "
    "application/x-rtp, encoding-name=H264 ! "
    "rtph264depay ! avdec_h264 ! "
    "videoconvert ! appsink"
)

# Option 2: Direct UDP (if above doesn't work)
# cap = cv2.VideoCapture('udp://127.0.0.1:5600')

cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

print("Opening camera feed...")
time.sleep(2)

if not cap.isOpened():
    print("❌ Camera feed not available")
    print("Trying alternative method...")
    cap = cv2.VideoCapture(0)  # Fallback to webcam for testing

def detect_red_target(frame):
    """
    Detect bright red box
    """
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Red color range (two ranges because red wraps around in HSV)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    
    # Create masks
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
    
    # Morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Get largest contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        # Filter noise
        if area > 100:  # Minimum area threshold
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(largest)
                
                return cx, cy, True, mask, (x, y, w, h), area
    
    return None, None, False, mask, None, 0

print("🔍 Starting detection test...")
print("Controls:")
print("  'q' - Quit")
print("  's' - Save screenshot")

frame_count = 0
detection_count = 0

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("⚠️ No frame received")
        time.sleep(0.1)
        continue
    
    frame_count += 1
    h, w = frame.shape[:2]
    
    # Detect target
    cx, cy, detected, mask, bbox, area = detect_red_target(frame)
    
    # Draw frame center
    cv2.circle(frame, (w//2, h//2), 10, (255, 0, 0), 2)
    cv2.line(frame, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 0, 0), 2)
    cv2.line(frame, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 0, 0), 2)
    
    if detected:
        detection_count += 1
        
        # Draw target info
        x, y, bw, bh = bbox
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
        cv2.circle(frame, (cx, cy), 15, (0, 255, 0), 3)
        
        # Draw line from center to target
        cv2.line(frame, (w//2, h//2), (cx, cy), (0, 255, 255), 2)
        
        # Calculate offsets
        offset_x = cx - w//2
        offset_y = cy - h//2
        distance = np.sqrt(offset_x**2 + offset_y**2)
        
        # Display info
        info_text = [
            f"✅ TARGET DETECTED",
            f"Center: ({cx}, {cy})",
            f"Offset X: {offset_x:.0f}px",
            f"Offset Y: {offset_y:.0f}px",
            f"Distance: {distance:.0f}px",
            f"Area: {area:.0f}px²"
        ]
        
        y_pos = 30
        for text in info_text:
            cv2.putText(frame, text, (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_pos += 25
        
        print(f"Frame {frame_count}: Target at ({cx}, {cy}) | Offset: ({offset_x:.0f}, {offset_y:.0f}) | Area: {area:.0f}")
    
    else:
        cv2.putText(frame, "❌ NO TARGET", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    # Detection rate
    if frame_count > 0:
        detection_rate = (detection_count / frame_count) * 100
        cv2.putText(frame, f"Detection: {detection_rate:.1f}%", (10, h - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # Show frames
    cv2.imshow('Camera Feed', frame)
    cv2.imshow('Red Mask', mask)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        cv2.imwrite(f'detection_screenshot_{int(time.time())}.jpg', frame)
        print("📸 Screenshot saved")

cap.release()
cv2.destroyAllWindows()

print(f"\n📊 Statistics:")
print(f"Total frames: {frame_count}")
print(f"Detections: {detection_count}")
print(f"Detection rate: {detection_count/frame_count*100:.1f}%")