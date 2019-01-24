# -*- coding: utf-8 -*-
"""
Created on Wed Jan 23 10:13:43 2019

@author: jschang

This script aims to get the point cloud stream as first step,
after that try to downsample it by applying Voxel Grid Filter
"""

# Import useful library
import numpy as np

# Import the realsense library
import pyrealsense2 as rs
# Import the pcl library
import pcl


try:  
    
    # Create a context object which owns the handle to all connected devices
    pipeline = rs.pipeline()
    
    # Configure the depth and color stream
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    pipeline.start()
    count=0
    
    while(count < 1):
        # Wait for a coherent pair of frames (depth, color)
        frames = pipeline.wait_for_frames()
        
        # Obtain the depth frame and color frame individaully
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        ######################################
        #   Generate PointCloud From Camera  #
        ######################################
                
        # Generate the pointcloud
        pc = rs.pointcloud()
        points = pc.calculate(depth_frame)
        pc.map_to(color_frame)
        
        # Pointcloud data to array
        v,t = points.get_vertices(), points.get_texture_coordinates()
        verts = np.asarray(v).view(np.float32).reshape(-1,3) #xyz
        textcoor = np.asarray(t).view(np.float32).reshape(-1,2) #uv
        
        
        
        
        ######################################
        #   Downsample the point cloud       #
        #     with Voxel Grid Filter         #
        ######################################       
        
        # Create a PCL pointcloud
        oriCloud = pcl.PointCloud(verts)
 
        # Create the Voxel Grid Filter Object
        vox = oriCloud.make_voxel_grid_filter()
        # Choose the voxel (leaf) size
        LEAF_SIZE = 0.01
        # Set the voxel size on the vox object
        vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
        

        # Call the voxel_grid_filter to obtain the downsampled cloud, called VGCloud (voxel_grid cloud)
        vgCloud = vox.filter()  
        
#        # Save the image for visualization
#        pcl.save(oriCloud, "oriCloud.pcd")
#        pcl.save(vgCloud, "vgCloud.pcd")
        
        
        
        

        
        
        
        
        
        count = count+1
        print(count)
        
        
        
    
    #Debug print the length of the data array
    print("Size of verts: ", verts.shape)
    print("Size of textcoor: ", textcoor.shape)
    

    
    
    
except Exception as e:
    print(e)
    pass
    