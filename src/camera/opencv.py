import cv2 
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs


# pipeline = rs.pipeline() # Create a pipeline
# pipeline.start() # Start streaming


# Camera
for i in range(0, 100):
    print(i)
    capture = cv2.VideoCapture(i)
    ret, frame = capture.read()
    print(f"==>> capture: {ret}")
# capture.set(cv2.CAP_PROP_FRAME_WIDTH,640)
# capture.set(cv2.CAP_PROP_FRAME_WIDTH,480)
capture = cv2.VideoCapture(6)
while True :
    # 33ms마다 반복문을 실행
    ret, frame = capture.read() # ret:camera 이상 여부, frame : 현재 시점의 frame
    print(f"==>> frame: {frame.shape}")
    new_img = cv2.resize(frame, (256, 256))
    print(f"==>> new_img: {new_img.shape}")
    cv2.imshow("VideoFrame", new_img)
    if cv2.waitKey(33) & 0xFF == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()




