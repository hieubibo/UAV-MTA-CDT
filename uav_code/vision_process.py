#!/usr/bin/env python3
from ultralytics import YOLO
import cv2
import os

model_path   = os.path.join("best.pt") #model đã train
output_video = os.path.join("videos\output_export.mp4")  # video kết quả