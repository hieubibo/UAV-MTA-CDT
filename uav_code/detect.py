import os
import cv2
import numpy as np
from ultralytics import YOLO


def centers(model_path="best.pt", cam_id=0, show=True, stop_event=None):
    base_dir = os.path.dirname(__file__)
    model_path = os.path.join(base_dir, model_path)

    model = YOLO(model_path)
    cap = cv2.VideoCapture(cam_id)

    if not cap.isOpened():
        raise RuntimeError("Không mở được camera")

    BOX_COLOR = (200, 0, 0)
    CENTER_COLOR = (0, 0, 255)
    CAM_CENTER_COLOR = (0, 255, 255)
    LINE_COLOR = (0, 255, 0)

    kf = cv2.KalmanFilter(4, 2)

    kf.measurementMatrix = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ], dtype=np.float32)

    kf.transitionMatrix = np.array([
        [1, 0, 1, 0],
        [0, 1, 0, 1],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1

    kf.statePost = np.zeros((4, 1), dtype=np.float32)
    kf.errorCovPost = np.eye(4, dtype=np.float32)

    first_measurement = True

    while True:
        if stop_event is not None and stop_event.is_set():
            break

        ok, frame = cap.read()
        if not ok:
            continue

        h, w = frame.shape[:2]
        cam_cx, cam_cy = w // 2, h // 2

        cv2.drawMarker(frame, (cam_cx, cam_cy), CAM_CENTER_COLOR, markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

        cx = cy = conf = None
        dx_raw = dy_raw = None

        r = model(frame, verbose=False)[0]
        b = r.boxes

        if b is not None and len(b):
            i = b.conf.argmax().item()
            conf = float(b.conf[i].item())
            x, y, bw, bh = b.xywh[i].tolist()

            cx, cy = int(x), int(y)
            bw, bh = int(bw), int(bh)

            x1, y1 = cx - bw // 2, cy - bh // 2
            x2, y2 = cx + bw // 2, cy + bh // 2

            cv2.rectangle(frame, (x1, y1), (x2, y2), BOX_COLOR, 2)
            cv2.circle(frame, (cx, cy), 4, CENTER_COLOR, -1)

            dx_raw = cx - cam_cx
            dy_raw = cy - cam_cy

            if first_measurement:
                kf.statePost = np.array([[dx_raw], [dy_raw], [0.0], [0.0]], dtype=np.float32)
                first_measurement = False

            kf.predict()

            meas = np.array([[dx_raw], [dy_raw]], dtype=np.float32)
            kf.correct(meas)

        else:
            if not first_measurement:
                kf.predict()

        if first_measurement:
            dx, dy = None, None
        else:
            state = kf.statePost 
            dx = round(float(state[0, 0]))  
            dy = round(float(state[1, 0]))  

        if cx is not None and dy is not None:
            cv2.line(frame, (cam_cx, cam_cy), (cx, cy), LINE_COLOR, 2)

        if dx is not None:
            cv2.putText(frame, f"dx={dx:.1f}px dy={dy:.1f}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, BOX_COLOR, 2)

        if conf is not None and b is not None and len(b):
            cv2.putText(frame, f"conf={conf:.2f}", (x1, max(0, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, BOX_COLOR, 2)

        if show:
            cv2.imshow("YOLO Detect", frame)
            if cv2.waitKey(2) & 0xFF == 27:
                break

        yield cx, cy, conf, dx, dy

    cap.release()
    cv2.destroyAllWindows()
