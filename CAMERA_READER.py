import socket
import numpy as np
import cv2
import time

# === CONFIG ===
UDP_IP = "0.0.0.0"          # Listen on all interfaces
UDP_PORT = 12345            # Must match ESP32
BUFFER_SIZE = 65536         # Max UDP packet size
CAMERA_FOV_DEG = 160        # Field of view
IMG_WIDTH = 160             # Must match ESP32
SHOW_VIDEO = True

# === Set up UDP socket ===
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(5.0)  # Adjust if needed

# === Frame buffer ===
jpeg_data = bytearray()

def get_green_bearing(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([40, 70, 70])
    upper_green = np.array([85, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            offset_px = cx - (IMG_WIDTH / 2.0)
            degrees_per_pixel = CAMERA_FOV_DEG / IMG_WIDTH
            bearing = offset_px * degrees_per_pixel
            return bearing, cx
    return None, None

print(f"Listening on UDP port {UDP_PORT}...")

try:
    while True:
        try:
            data, _ = sock.recvfrom(BUFFER_SIZE)
            jpeg_data.extend(data)

            # Try to find full JPEG
            start = jpeg_data.find(b'\xff\xd8')  # JPEG start
            end = jpeg_data.find(b'\xff\xd9')    # JPEG end

            if start != -1 and end != -1 and end > start:
                jpg = jpeg_data[start:end+2]
                jpeg_data = jpeg_data[end+2:]

                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                frame = cv2.resize(frame, (IMG_WIDTH, 120))
                bearing, cx = get_green_bearing(frame)

                if bearing is not None:
                    print(f"[✓] Green Bearing: {bearing:.1f} degrees")
                    if SHOW_VIDEO:
                        cv2.circle(frame, (cx, 60), 5, (0, 255, 0), -1)
                        cv2.line(frame, (IMG_WIDTH//2, 0), (IMG_WIDTH//2, 120), (255, 255, 255), 1)
                        cv2.putText(frame, f"{bearing:.1f} deg", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                else:
                    print("[ ] No green object detected")

                if SHOW_VIDEO:
                    cv2.imshow("UDP Stream", frame)
                    if cv2.waitKey(1) & 0xFF == 27:
                        break
        except socket.timeout:
            print("UDP timeout. No data.")
        except Exception as e:
            print("Error:", e)

except KeyboardInterrupt:
    print("Exiting...")

cv2.destroyAllWindows()
sock.close()
