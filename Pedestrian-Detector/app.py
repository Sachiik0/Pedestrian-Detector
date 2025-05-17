import cv2
import time
from ultralytics import YOLO
from luma.core.interface.serial import spi, noop
from luma.led_matrix.device import max7219
from luma.core.render import canvas
from PIL import ImageFont

# --- CONFIGURATION ---
LED_CASCADE = 4
LED_ORIENTATION = -90
PEDESTRIAN_CLASSES = ["person"]
VEHICLE_CLASSES = ["car", "truck", "bus", "motorbike", "bicycle"]
YOLO_MODEL = "yolov8n.pt"
YOLO_CONF = 0.3
FPS = 30
FRAME_TIME = 1 / FPS

# --- INIT YOLO ---
model = YOLO(YOLO_MODEL)

# --- INIT LED MATRIX ---
serial = spi(port=0, device=0, gpio=noop())
device = max7219(serial, cascaded=LED_CASCADE, block_orientation=LED_ORIENTATION, rotate=0)
font = ImageFont.load_default()

def show_led(msg):
    with canvas(device) as draw:
        draw.text((1, -2), msg, fill="white", font=font)

# --- CAMERA UTILITIES ---
def find_available_camera(start=0, end=10):
    for i in range(start, end):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            cap.release()
            return i
        cap.release()
    return None

def init_camera(index, width=640, height=480, fps=10):
    cam = cv2.VideoCapture(index)
    cam.set(cv2.CAP_PROP_FPS, fps)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cam

# --- DETECTION UTILITY ---
def detect_objects(frame):
    results = model.predict(frame, imgsz=640, conf=YOLO_CONF)
    return results[0].boxes, results[0].names

def process_detections(frame, boxes, class_names, detect_classes, color):
    detected = False
    for box in boxes:
        cls = int(box.cls[0])
        label = class_names[cls].lower()
        if label in detect_classes:
            detected = True
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return detected

# --- MAIN EXECUTION ---
ped_index = find_available_camera(0, 10)
if ped_index is None:
    print("No pedestrian camera found.")
    exit()

vehicle_index = find_available_camera(ped_index + 2, ped_index + 6)

cam_ped = init_camera(ped_index)
print(f"[Pedestrian Cam] /dev/video{ped_index} | FPS: {cam_ped.get(cv2.CAP_PROP_FPS)}")

cam_vehicle = init_camera(vehicle_index, fps=10) if vehicle_index is not None else None
if cam_vehicle:
    print(f"[Vehicle Cam] /dev/video{vehicle_index} | FPS: {cam_vehicle.get(cv2.CAP_PROP_FPS)}")
else:
    print("[Vehicle Cam] Not Found")

while True:
    start_time = time.time()
    ret_ped, frame_ped = cam_ped.read()
    if not ret_ped:
        print("Pedestrian camera read failed.")
        break

    ret_car, frame_car = False, None
    if cam_vehicle:
        ret_car, frame_car = cam_vehicle.read()

    # YOLO detection on pedestrian camera
    boxes, class_names = detect_objects(frame_ped)
    pedestrian_detected = process_detections(frame_ped, boxes, class_names, PEDESTRIAN_CLASSES, (0, 255, 0))
    vehicle_detected = process_detections(frame_ped, boxes, class_names, VEHICLE_CLASSES, (0, 0, 255))

    # YOLO detection on vehicle camera
    if cam_vehicle and ret_car:
        boxes_v, class_names_v = detect_objects(frame_car)
        vehicle_detected |= process_detections(frame_car, boxes_v, class_names_v, VEHICLE_CLASSES, (0, 0, 255))

    # LED Logic
    if pedestrian_detected and vehicle_detected:
        show_led("STOP")
        print("[STATUS] Pedestrian + Vehicle -> STOP")
    elif pedestrian_detected:
        show_led("GO")
        print("[STATUS] Pedestrian only -> GO")
    else:
        show_led("OFF")
        print("[STATUS] No pedestrian -> OFF")

    # Display windows
    cv2.imshow("Pedestrian Camera", frame_ped)
    if cam_vehicle and ret_car:
        cv2.imshow("Vehicle Camera", frame_car)

    if cv2.waitKey(1) == 27:  # ESC
        break

    # Maintain framerate
    elapsed = time.time() - start_time
    time.sleep(max(0, FRAME_TIME - elapsed))

# Cleanup
cam_ped.release()
if cam_vehicle:
    cam_vehicle.release()
cv2.destroyAllWindows()
