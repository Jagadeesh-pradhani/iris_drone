import sys
import os

# Add yolov7 folder to the Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
yolov7_path = os.path.abspath(os.path.join(current_dir, '../../../../easy_yolov7'))
sys.path.insert(0, yolov7_path)
weights_path = os.path.join(yolov7_path, 'coco.weights')
yaml_path = os.path.join(yolov7_path, 'coco.yaml')

from algorithm.object_detector import YOLOv7
from utils.detections import draw
import json
import cv2

def main():
    yolov7 = YOLOv7()
    yolov7.load(weights_path, classes=yaml_path, device='cpu') # use 'gpu' for CUDA GPU inference
    image = cv2.imread('image.jpg')
    detections = yolov7.detect(image)
    detected_image = draw(image, detections)
    cv2.imwrite('detected_image.jpg', detected_image)
    print(json.dumps(detections, indent=4))

if __name__ == '__main__':
    main()