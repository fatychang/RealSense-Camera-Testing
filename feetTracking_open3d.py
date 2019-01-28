# -*- coding: utf-8 -*-
"""
Created on Mon Jan 28 11:32:20 2019

@author: jschang
"""

# First import library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Import open3d for point cloud image processing
import open3d
# Import time package
import time


####################################################
##           Initialization -                     ##      
##  Read the video and make ready the pipeline    ##
###################################################

# Flag setting
SAVE_IMAGE = False
IS_DEBUG = False


# .bag file location
FILE_LOC = "D:\\Jen\\Projects\\RealSense Camera\\Recordings\\feetTest1.bag"
#FILE_LOC = "D:\\Jen\\Projects\\RealSense Camera\\Recordings\\d415_1500.bag"

# Create the context object that holds the handle of all the connected devices
pipeline = rs.pipeline()

# Create the config object and configure the stream
cfg = rs.config()
# Tell config that we will use a recorded device from filem to be used by the pipeline through playback.
rs.config.enable_device_from_file(cfg, FILE_LOC)
# Configure the pipeline to stream the depth stream (resolution, format, and frame rate)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming from file and obtain the returned profile
profile = pipeline.start(cfg)


# Obtain the depth stream profile and camera intrinsics
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()

# Get the width and height of the frame from the camera intrinsics
w, h = depth_intrinsics.width, depth_intrinsics.height

# Create pointcloud object
pc = rs.pointcloud()

# Create realsense colorizer object
colorizer = rs.colorizer()


#######################
#    Stream loop      #
#######################

# Process one frame only
LOOP_TIME = 1
ctr=0
while ctr < LOOP_TIME:

    # Get the frames from pipeline
    frames = pipeline.wait_for_frames()
    
    # Get depth frame
    depth_frame = frames.get_depth_frame()
    
    # Get the depth frame data
    depth_image = np.asanyarray(depth_frame.get_data())
    
    # Get the depth color map
    depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    
    # Obtain the mapped frame and color_source
    mapped_frame, color_source = depth_frame, depth_colormap    
    
    # Calculate the points from the depth_frame
    points = pc.calculate(depth_frame)
    
    # Pointcloud data to array
    v,t = points.get_vertices(), points.get_texture_coordinates()
    verts = np.asarray(v).view(np.float32).reshape(-1,3) #xyz
    texcoords = np.asarray(t).view(np.float32).reshape(-1,2) #uv
    
    # Generate the open3d point cloud
    pcd = open3d.PointCloud()
    pcd.points = open3d.Vector3dVector(verts)
    
#    # Visualize the point cloud
#    open3d.draw_geometries([pcd])
    
    ######################################
    #   Downsample the point cloud       #
    #     with Voxel Grid Filter         #
    ######################################
    
    now = time.time()
    # Set the voxel size for filtering
    voxel_size = 0.01
    
    # Apply voxel grid filtering for downsampling
    downpcd = open3d.voxel_down_sample(pcd, voxel_size)
    
    # Downsampling time
    dt = time.time() - now
#    print("Downsampling time: %10.9f", dt)
    
#    # Visuzlize the downsampled point cloud
#    open3d.draw_geometries([downpcd])
    






























