# RealSense-Camera-Testing
This project contains the codes to manipulate the Intel RealSense Camera and simple image clustering, segmentation and regonition via Python.
PCL or open3d are the libraries we used to process the point cloud data. 

Noted: This is an ongoing project. New features are developing and there are some bugs need to be fixed.

## Read_Bag_Video.py
The "Read_Bad_Video" allows the program to open a simple window displaying the recorded video ( the video is recorded via the Intel Realsense Viewer). It is modified from the original sample code obtained from (https://github.com/IntelRealSense/librealsense/blob/development/wrappers/python/examples/read_bag_example.py). You may refer to it for more details.

How to Run?
1. First, you will need to change the file path (FILE_LOC) based on the location where you store the .bag file.
2. Second, configure the pipeline with the corresponding stream type (depth or color), resolution ( 640*480 in this case), format, and frame rate which are same as the recording video. Otherwise you will encounter errors while running.
3. Run the script.
4. Press escape key to exit the program.


## feetTracking.py
The script aims to apply image pre-processing techniques which include downsampling, image cropping, and ground removal.
The PCL library is implemented to achieve the above function.
However, the point cloud visualization haven't finish.

* The script at first read the bag file and configure the pipeline stream in order to extract the frame. (please noted: when configuring, the resolution must matchs the recorded stream, otherwise, an error will occurs)

* Second, apply downsampling, image cropping, ground and outlier removel technique via the pcl library. The user can choose whether a .pcl file should be saved after each step.

* Downsampling: A voxel grip filter is applied to take the average value inside the defined voxel (param: LEAF_SIZE)
* Image Cropping: a passthrough fileter is designed for cutting the image outside the boundary along a certain axis
* Ground Segmentation: Implements the RANSAC alog. to filter out the target shape (PLANE)
* Outlier Removal: A statistical outlier filter removes the outlier based on the number of neighbors within a defined boundary.
* The result of each step can be saved as a indepentent .pcd file for further visualization via the pcl_viewer.exe

## PCL_VIEWER_RELEASE.EXE
This is a exe file for point cloud visualization.
How?
1. Locate the pcl_viewer_release.exe and OpenNI2.dll file (should be under C:\Program Files\PCL x.x.x\bin).
2. Copy the two files along with the .pcd file to same folder.
3. Open the terminal and navigate to the file where the files are stored.
4. Key in the following command:
  "pcl_viewer_release file_name.pcd"
