# calibration_module 
This repository contains toolbox and data management utilities for sensor calibration. All the extrinsic parameters of the sensors are calibrated to a VLP-16 LiDAR.

## Dependencies
1. Working Python 3.6 environment with ROS. This can be a bit difficult. See the README under the [objection detection repo](https://bitbucket.org/cmusubt/object_detection/src/master/) to see how to set this up.
2. Install gnupg in Python3 for rosbag compatibility. Use the following commandline input.
```bash
$ sudo pip3 install python-gnupg
```

## Usage
### Data Recording
1. Get a checker board. Tested with the one in Mavlab with square size of 119mm*119mm. If you are calibrating a thermal camera, leave the calibration board under sunlight or heat source for 5 minutes, then the thermal camera should be able to detect the checkerboard pattern.
2. Record the data into bag files. Make sure the checkerboard is within the FOV of the camera. Stand 3 to 6 meters away from the sensors. Each bag file should contain one static checkerboard pose. Record each pose bag for 10 seconds. Record 10-15 poses for each camera/LiDAR pair.
3. Organize the bag files for different cameras into different folders. For example: /front_camera/<pose1.bag, pose2.bag, pose3.bag, ...>, /left_camera/<pose1.bag, pose2.bag, pose3.bag, ...>.

### Preparing for calibration
1. Give scripts excutable flag:
```bash
$ cd <calibration_module_repo>/util
$ sudo chmod +x create_file_structure.sh save_data_from_bag.sh save_single_image_from_bag.py
```
2. Edit ```create_file_structure.sh``` for camera topic info. Add or delete the following topic names:
```bash
# Create directory for each camera direction
FRONT_CAM_TOPIC=rs_front
BACK_CAM_TOPIC=rs_back
RIGHT_CAM_TOPIC=rs_right
LEFT_CAM_TOPIC=rs_left
THERMAL_TOPIC=thermal
```
Create data file structure for calibration script. Use the following script:
```bash
$ ./create_file_structure.sh
```
3. Load bag files into corresponding directories. For example, bag files for front realsense camera should go to /data/rs_front/bag.
4. Start a roscore in a terminal.
```bash
$ roscore
```
5. Extract data from bag files. This will take about 5 minutes depending on the number of bag files:
```bash
$ ./save_data_from_bag.sh
```
6. Examine the extracted image data. Delete any (image, pointcloud) pair if the calibration target is invalid (out of FOV, large angle, etc).

### Camera intrinsic calibration
Use ```/tool_box/intrinsic_calibration``` to calibrate intrinsic paramters for each camera. [Here](http://www.vision.caltech.edu/bouguetj/calib_doc/) is the link to the tutorial of the toolbox.

Save the ```Calib_Result.mat``` file in the image folder.

Note1: This toolbox is relatively difficult to use, however the result format is necessary for LiDAR/camera extrinsics calibration. If you are only calibrating the intrinsics paramters of camera, use ```/tool_box/intrinsic_calibration_only```.

Note2: The toolbox cannot properly save the result if there already exists files with name ```Calib_Result.mat``` in the matlab path.

### LiDAR camera extrinsic calibration
Use ```/tool_box/extrinsic_calibration``` to calibrate extrinsic paramters between one camera and VLP-16. [Here](http://www.cs.cmu.edu/~ranjith/lcct.html) is the link to the original toolbox and [here](https://github.com/zhixy/Laser-Camera-Calibration-Toolbox) is the link to the modified toolbox to improve LiDAR plane selection.

I also made some modifications to support sparse point cloud for VLP-16, but the procedure is basically the same. I suggest to read up both links above for more details.

Save the calibration result in the xyz folder.

### Checking the result
In xyz folder, run matlab script ```project_to_cam_main.m``` from ```/util``` to show the projected lidar points on the image.

See if the discontinuities of the lidar points are reasonable in the image (e.g. edge of the checker board). 