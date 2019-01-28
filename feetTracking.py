# -*- coding: utf-8 -*-
"""
Created on Thu Jan 24 11:46:45 2019

This script aims to implement image clustering, segmentation technique
to achieve the feetTracking function

@author: jschang
"""

# First import library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Import PCL for filtering
import pcl
# Import point processing toolkit (pptk) for visualizing point clouds
import pptk
# Import time packages for pointcloud rendering
import time
# Import pointcloud viewer.py
import pointcloudViewer



#####################################
#   Function for displaying stream  #
#   (UNFINISHED)                    #
#####################################
def imageShow(pipeline):
     
    # Colorize the depth frame to jet colormap
    depth_color_frame = rs.colorizer().colorize(depth_frame)
        
    # Convert depth_frame to numpy array to render image in opencv
    depth_color_image = np.asarray(depth_color_frame.get_data())
        
    # Render image in opencv window
    cv2.imshow("playback", depth_color_image)
    key = cv2.waitKey(1)
        
    # Exit program when the escape is pressed
    if key==27:
        cv2.destoryAllWindows()
    return




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

# Create AppState object for pointcloud visualization
state = pointcloudViewer.AppState()

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

# Initialize the opencv window
pointcloudViewer.window_init(state)
#cv2.namedWindow("Playback", cv2.WINDOW_AUTOSIZE)

# Create param 'out' to store the frame data
out = np.empty((h, w, 3), dtype=np.uint8)



#######################
#    Stream loop      #
#######################

# Extract a paticular frame
#ctr=0
#NUM_OF_FRAMES = 75
#while ctr < NUM_OF_FRAMES:
#    # Get the frames from pipeline
#    frames = pipeline.wait_for_frames()    
#    # Get depth frame
#    depth_frame = frames.get_depth_frame()
#    ctr = ctr + 1

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
    

    
    ######################################
    #   Downsample the point cloud       #
    #     with Voxel Grid Filter         #
    ######################################
    
    # Create a PCL pointcloud
    oriCloud = pcl.PointCloud(verts)

 
    # Starting time for downsampling
    now = time.time()
    
    # Create the Voxel Grid Filter Object
    vox = oriCloud.make_voxel_grid_filter()
    # Choose the voxel (leaf) size
    LEAF_SIZE = 0.01
    # Set the voxel size on the vox object
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    
    # Call the voxel_grid_filter to obtain the downsampled cloud, called VGCloud (voxel_grid cloud)
    vgCloud = vox.filter()  
    
    
    # Downsampling time
    dt = time.time() - now   
    
                
    # Debug
    if IS_DEBUG:
        # Print the size of the original point cloud
        print("The size of the original pointcloud: ", oriCloud.size) 
        # Print process time
        print("Downsampleing time: %10.9f", dt)
        # Print the size of the downsampled point cloud
        print("The size of the downsampled pointcloud: ", vgCloud.size)  
    
    
    # Save the image for visualization
    if SAVE_IMAGE:
        pcl.save(oriCloud, "oriCloud.pcd")
        pcl.save(vgCloud, "vgCloud.pcd")    
    
    
    
    
    ################################
    #   Apply Passthrough Filter   #
    #     (crop the image)         #
    ################################
    
    # Starting time for downsampling
    now1 = time.time()
    
    
    # Create a passthrough filter object
    passthrough = vgCloud.make_passthrough_filter()
        
    # Assign axis and range to the passthrough filter()
    FILTER_AXIS = 'z'
    passthrough.set_filter_field_name(FILTER_AXIS)
    AXIS_MIN = 0
    AXIS_MAX = 2
    passthrough.set_filter_limits(AXIS_MIN, AXIS_MAX)
        
    # Call the passthrough filter to obtain the resultant pointcloud
    ptCloud = passthrough.filter()
     
    # Downsampling time
    dt1 = time.time() - now1 


    # Debug
    if IS_DEBUG:
        # Print the size of the cropped point cloud
        print("The size of the cropped pointcloud: ", ptCloud.size)
        # Print process time
        print("Cropping time: %10.9f", dt1)

        
        
        
    # Save the image for visualization
    if SAVE_IMAGE:
        pcl.save(ptCloud, "passthroughCloud.pcd")
    
    
    
    
    ###################################
    #   Ground Segmentation (remove)  #
    #          via RANSAC             #
    ###################################
    
    # Starting time for ground segmentation
    now2 = time.time()
    
    # Create the segmentation object
    seg = ptCloud.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    #seg.set_model_type(pcl.SAC_RANSAC)
                       
    # Max distance for the point to be consider fitting this model
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Obtain a set of inlier indices ( who fit the plane) and model coefficients
    inliers, coefficients = seg.segment()
    
    # Extract Inliers obtained from previous step
    gdRemovedCloud = ptCloud.extract(inliers, negative=True)

    # Ground segmentation time
    dt2 = time.time() - now2
    
   
    
    # Debug
    if IS_DEBUG:
        # Print the size of the ground removed point cloud
        print("The size of the ground removed pointcloud: ", gdRemovedCloud.size)
        # Print process time
        print("Ground segmentation time: %10.9f", dt2)
       
    
    
    # Save the image for visualization
    if SAVE_IMAGE:
        pcl.save(gdRemovedCloud, "gdRemovedCloud.pcd")
    
    
    
    
    #################################
    #   Outlier removal Filter-     #
    #  Statistical Outlier Removal  #
    #################################
    
    # Starting time for outlier removal 
    now3 = time.time()
    
    
    # Create a statistical outlier filter object
    outlier = gdRemovedCloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier.set_mean_k(50)
    
    # Set threshold scale factor
    outlier_threshold = 1.0

    # Eliminate the points whose mean distance is larger than global
    # (global dis = mean_dis + threshold * std_dev)               
    outlier.set_std_dev_mul_thresh(outlier_threshold)

    # Apply the statistical outlier removal filter
    olRemovedCloud = outlier.filter()

    # Outlier removal time
    dt3 = time.time() - now3
    


    # Debug
    if IS_DEBUG:
        # Print the size of the ground removed point cloud
        print("The size of the outlier removed pointcloud: ", olRemovedCloud.size)   
        # Print process time
        print("Outlier removal time: %10.9f", dt3)


    # Save the image for visualization
    if SAVE_IMAGE:
        pcl.save(olRemovedCloud, "outlierRemovedCloud.pcd")
    
    
#    ##############################
#    #   PointCloud Visualization #
#    #       (Unfinished)         #
#    ##############################       
#    
#    # Convert the pcl pointcloud object to array type
#    displayCloud = olRemovedCloud
#    display_verts = np.asarray(displayCloud).view(np.float32).reshape(-1,3) #xyz
#    
#    
#    # Render
#    #now = time.time()
#    
#    #out.fill(0) #fill the 3d array with zeros
#    
#    # Draw the grid
#    #pointcloudViewer.grid(state, out, (0, 0.5, 1), size=1, n=10)
#    # Draw the axes
#    #pointcloudViewer.axes(out, pointcloudViewer.view([0, 0, 0], state), state.rotation, size=0.1, thickness=1)
#    
#    # Create a PCL pointcloud viewer
#    viewer = pcl.pcl_visualization.CloudViewing('Cloud Viewer')
#    
#        
#    # Visualize the pointcloud
#    #viewer = pptk.viewer(display_verts)
#    
#    
#    # Draw the pointcloud
#    #pointcloudViewer.pointcloud(state, out, display_verts, texcoords, color_source, painter=False)
#    
#    # Listen mouse state
#    #if any(state.mouse_btns):
#        #pointcloudViewer.axes(out, pointcloudViewer.view(state.pivot), state.rotation, thickness=4)
#    
#    # Change of time
#    #dt = time.time()-now
#    
#    # Show the image
#    cv2.imshow(state.WIN_NAME, out)
#    key = cv2.waitKey(1)
    
    
    
    
    # frame counter update
    #ctr=ctr+1

# Stop streaming
pipeline.stop()
