import cv2
import socket
import time
from ultralytics import YOLO

# ==== CONFIG ====
ESP_IP = "10.4.2.58"     # <-- replace with your ESP32 IP
ESP_UDP_PORT = 5005
STREAM_URL = f"http://{ESP_IP}:81/stream"

MODEL_PATH = "best.pt"   # your trained YOLOv8n weights
OBSTACLE_CLASS_NAME = "obstacle"  # or whatever class name you used

# Command rate limiting
SEND_INTERVAL = 0.1  # seconds

def get_region(x_center, width, left_thr=0.33, right_thr=0.66):
    """
    Map horizontal position to LEFT / CENTER / RIGHT.
    x_center: center of bbox
    width: frame width
    """
    rel = x_center / width
    if rel < left_thr:
        return "LEFT"
    elif rel > right_thr:
        return "RIGHT"
    else:
        return "CENTER"

def main():
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Load YOLO model
    model = YOLO(MODEL_PATH)

    # Open MJPEG stream
    cap = cv2.VideoCapture(STREAM_URL)

    if not cap.isOpened():
        print("Failed to open video stream from ESP32")
        return

    last_sent_cmd = None
    last_sent_time = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            time.sleep(0.1)
            continue

        h, w = frame.shape[:2]

        # Run YOLO inference
        results = model(frame, imgsz=320, verbose=False)[0]

        # Find obstacle bboxes
        best_box = None
        best_area = 0

        for box in results.boxes:
            cls_id = int(box.cls[0].item())
            cls_name = model.names[cls_id]

            if cls_name != OBSTACLE_CLASS_NAME:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            area = (x2 - x1) * (y2 - y1)

            if area > best_area:
                best_area = area
                best_box = (x1, y1, x2, y2)

        # Decide region
        if best_box is None:
            cmd = "NONE"
        else:
            x1, y1, x2, y2 = best_box
            x_center = 0.5 * (x1 + x2)
            cmd = get_region(x_center, w)

            # Draw bounding box on frame
            color = (0, 255, 0)  # green
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.line(frame, (int(x_center), 0), (int(x_center), h), color, 1)
            cv2.putText(frame, cmd, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # Rate-limit UDP sending & avoid spamming same command constantly
        now = time.time()
        if cmd != last_sent_cmd or (now - last_sent_time) > SEND_INTERVAL:
            msg = cmd.encode("utf-8")
            sock.sendto(msg, (ESP_IP, ESP_UDP_PORT))
            last_sent_cmd = cmd
            last_sent_time = now

        # Overlay info
        cv2.putText(frame, f"Cmd: {cmd}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        cv2.imshow("YOLO View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    sock.close()

if __name__ == "__main__":
    main()
