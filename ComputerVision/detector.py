#install tensorflow, opencv-python, keras, imageAI
import sys
from imageai.Detection import VideoObjectDetection
import os
import cv2

execution_path = os.getcwd()
camera = cv2.VideoCapture(0)

detector = VideoObjectDetection()

#C:\Users\alber\Desktop\objectDetection

model_path = r'ProjectDirectory\models\yolo-tiny.h5'
input_path = r'ProjectDirectory\input\cars.mp4'
output_path = r'ProjectDirectory\output\street.mp4'

detector.setModelTypeAsTinyYOLOv3()
detector.setModelPath(model_path)
detector.loadModel()

#camera_input=camera
video_path = detector.detectObjectsFromVideo(input_file_path=input_path, output_file_path=output_path, frames_per_second=29, log_progress=True)
print(video_path)
