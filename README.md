# RealSense-Camera-Testing
This project contains the codes to manipulate the Intel RealSense Camera and simple image clustering, segmentation and regonition

# Read_Bag_Video
The "Read_Bad_Video" allows the program to open a simple window displaying the recorded video ( the video is recorded via the Intel Realsense Viewer). It is modified from the original sample code obtained from (https://github.com/IntelRealSense/librealsense/blob/development/wrappers/python/examples/read_bag_example.py). You may refer to it for more details.

How to Run?
1. First, you will need to change the file path (FILE_LOC) based on the location where you store the .bag file.
2. Second, configure the pipeline with the corresponding stream type (depth or color), resolution ( 640*480 in this case), format, and frame rate which are same as the recording video. Otherwise you will encounter errors while running.
3. Run the script.
4. Press escape key to exit the program.
