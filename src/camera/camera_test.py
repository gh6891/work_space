import cv2 
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs


pipeline = rs.pipeline() # Create a pipeline
pipeline.start() # Start streaming

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_frame_width = color_frame.get_width()
        color_frame_height = color_frame.get_height()
        print(f"==>> color_frame_width: {color_frame_width}")
        print(f"==>> color_frame_height: {color_frame_height}")
        
        data = frames.get_data()
        print(f"==>> data: {data}")
        data_size = frames.get_data_size()
        print(f"==>> data_size: {data_size}")

        # depth_frame = frames.get_depth_frame()
        # if not depth_frame:
        #     continue

        # width, height = depth_frame.get_width(), depth_frame.get_height()
        # dist = depth_frame.get_distance(width // 2, height // 2)
        # print(f"The camera is facing an object {dist:.3f} meters away", end="\r")

finally:
    pipeline.stop() # Stop streaming