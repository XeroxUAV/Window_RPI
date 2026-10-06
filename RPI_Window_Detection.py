import cv2
import time
import torch

from ultralytics import YOLO
from picamera2 import Picamera2


# ============================================================
# SETTINGS
# ============================================================

MODEL_PATH = "/home/pi/best.pt"

CONFIDENCE = 0.5

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


# ============================================================
# LOAD YOLO
# ============================================================

print("[INFO] Loading YOLO model...")

device = "cuda" if torch.cuda.is_available() else "cpu"

model = YOLO(MODEL_PATH)

print(f"[INFO] Device: {device}")
print("[INFO] Model loaded")


# ============================================================
# RASPBERRY PI CAMERA
# ============================================================

print("[INFO] Starting Raspberry Pi camera...")

picam2 = Picamera2()

camera_config = picam2.create_preview_configuration(
    main={
        "size": (
            CAMERA_WIDTH,
            CAMERA_HEIGHT
        ),
        "format": "RGB888"
    }
)

picam2.configure(camera_config)

picam2.start()

time.sleep(2)

print("[INFO] Camera started")
print("[INFO] Starting window detection...")


# ============================================================
# MAIN LOOP
# ============================================================

while True:

    # --------------------------------------------------------
    # Capture frame
    # --------------------------------------------------------

    frame = picam2.capture_array()

    # Picamera2 -> RGB
    # OpenCV -> BGR

    frame = cv2.cvtColor(
        frame,
        cv2.COLOR_RGB2BGR
    )


    # --------------------------------------------------------
    # YOLO DETECTION
    # --------------------------------------------------------

    results = model.predict(
        source=frame,
        conf=CONFIDENCE,
        device=device,
        verbose=False
    )


    # --------------------------------------------------------
    # FIND BEST WINDOW
    # --------------------------------------------------------

    best_detection = None

    best_area = 0


    for result in results:

        if result.boxes is None:
            continue


        for box in result.boxes:

            confidence = float(box.conf[0])

            if confidence < CONFIDENCE:
                continue


            # Bounding box

            x1, y1, x2, y2 = map(
                int,
                box.xyxy[0]
            )


            width = x2 - x1
            height = y2 - y1

            area = width * height


            # Keep the largest detection

            if area > best_area:

                best_area = area

                best_detection = {
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "width": width,
                    "height": height,
                    "confidence": confidence
                }


    # ========================================================
    # PROCESS WINDOW
    # ========================================================

    if best_detection is not None:

        x1 = best_detection["x1"]
        y1 = best_detection["y1"]
        x2 = best_detection["x2"]
        y2 = best_detection["y2"]

        width = best_detection["width"]
        height = best_detection["height"]

        confidence = best_detection["confidence"]


        # ----------------------------------------------------
        # WINDOW CENTER
        # ----------------------------------------------------

        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2


        # ----------------------------------------------------
        # IMAGE CENTER
        # ----------------------------------------------------

        image_center_x = CAMERA_WIDTH // 2
        image_center_y = CAMERA_HEIGHT // 2


        # ----------------------------------------------------
        # ERROR FROM IMAGE CENTER
        # ----------------------------------------------------

        error_x = center_x - image_center_x
        error_y = center_y - image_center_y


        # ----------------------------------------------------
        # DRAW WINDOW
        # ----------------------------------------------------

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2
        )


        # ----------------------------------------------------
        # DRAW WINDOW CENTER
        # ----------------------------------------------------

        cv2.circle(
            frame,
            (center_x, center_y),
            6,
            (0, 0, 255),
            -1
        )


        # ----------------------------------------------------
        # DRAW LINE FROM IMAGE CENTER TO WINDOW CENTER
        # ----------------------------------------------------

        cv2.line(
            frame,
            (
                image_center_x,
                image_center_y
            ),
            (
                center_x,
                center_y
            ),
            (255, 0, 0),
            2
        )


        # ----------------------------------------------------
        # DRAW IMAGE CENTER
        # ----------------------------------------------------

        cv2.circle(
            frame,
            (
                image_center_x,
                image_center_y
            ),
            5,
            (255, 0, 0),
            -1
        )


        # ====================================================
        # INFORMATION
        # ====================================================

        cv2.putText(
            frame,
            "WINDOW DETECTED",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )


        cv2.putText(
            frame,
            f"Confidence: {confidence:.2f}",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )


        cv2.putText(
            frame,
            f"Center: ({center_x}, {center_y})",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )


        cv2.putText(
            frame,
            f"Size: {width} x {height}",
            (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )


        cv2.putText(
            frame,
            f"Error X: {error_x}",
            (20, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )


        cv2.putText(
            frame,
            f"Error Y: {error_y}",
            (20, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )


        # Print detection to terminal

        print(
            f"[WINDOW] "
            f"Center=({center_x},{center_y}) "
            f"Size={width}x{height} "
            f"Confidence={confidence:.2f}"
        )


    else:

        # ----------------------------------------------------
        # NO WINDOW
        # ----------------------------------------------------

        cv2.putText(
            frame,
            "WINDOW NOT DETECTED",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )


    # ========================================================
    # IMAGE CENTER CROSSHAIR
    # ========================================================

    cv2.line(
        frame,
        (
            CAMERA_WIDTH // 2,
            0
        ),
        (
            CAMERA_WIDTH // 2,
            CAMERA_HEIGHT
        ),
        (255, 0, 0),
        1
    )

    cv2.line(
        frame,
        (
            0,
            CAMERA_HEIGHT // 2
        ),
        (
            CAMERA_WIDTH,
            CAMERA_HEIGHT // 2
        ),
        (255, 0, 0),
        1
    )


    # ========================================================
    # SHOW
    # ========================================================

    cv2.imshow(
        "Window Detection",
        frame
    )


    # ESC = exit

    key = cv2.waitKey(1) & 0xFF

    if key == 27:
        break


# ============================================================
# CLEANUP
# ============================================================

cv2.destroyAllWindows()

picam2.stop()

print("[INFO] Program stopped")