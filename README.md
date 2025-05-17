# Pedestrian-Detector
This project is a commissioned work for Adamson University 3rd Year BS Computer Science students as part of their Capstone 1. It features real-time pedestrian and vehicle detection with LED matrix signaling using YOLO and Raspberry Pi.

YOLOv8 PEDESTRIAN-VEHICLE DETECTION SYSTEM WITH LED DISPLAY
============================================================

This system detects pedestrians and vehicles using two USB cameras and YOLOv8,
and displays a status message ("STOP", "GO", or "OFF") on an 8x32 LED matrix display.

------------------------------------------------------------
HARDWARE REQUIRED:
------------------------------------------------------------
- Raspberry Pi (with SPI enabled)
- 8x32 LED matrix (MAX7219, 4 cascaded)
- 1 or 2 USB Cameras (1 for pedestrian detection, 1 for vehicle detection)
- Internet connection (for initial setup)

------------------------------------------------------------
SOFTWARE REQUIREMENTS:
------------------------------------------------------------
- Python 3.8+
- Python packages:
    - ultralytics
    - opencv-python
    - luma.led_matrix
    - Pillow

To install dependencies, run:
    pip install ultralytics opencv-python luma.led_matrix Pillow

------------------------------------------------------------
MODEL FILE:
------------------------------------------------------------
This system uses YOLOv8n for lightweight and fast detection.
Download the model using:

    wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

Place the `yolov8n.pt` file in the same folder as the main script.

------------------------------------------------------------
HOW TO USE (on Raspberry Pi):
------------------------------------------------------------
1. Boot up your Raspberry Pi.
2. Open the Terminal.
3. Navigate to your project folder:

       cd Ped_Detection

4. Run the program:

       python app.py

The system will:
- Automatically find and activate connected USB cameras.
- Run YOLOv8 detection on camera feeds.
- Display results on the LED matrix based on object presence.

------------------------------------------------------------
LED MATRIX DISPLAY LOGIC:
------------------------------------------------------------
- "STOP"  --> A pedestrian AND vehicle are detected.
- "GO"    --> Only a pedestrian is detected.
- "OFF"   --> No pedestrian is detected.

------------------------------------------------------------
TROUBLESHOOTING:
------------------------------------------------------------
- If the LED matrix doesn't light up, make sure SPI is enabled:
    Run `sudo raspi-config`, go to Interface Options > SPI > Enable
- If the camera doesn't show up, make sure it's plugged in properly.
- You can reduce the frame size or switch to a faster model for better performance.

------------------------------------------------------------
CREDITS:
------------------------------------------------------------
- YOLOv8 by Ultralytics
- Luma.LED Matrix by rm-hull
- OpenCV

MIT License
