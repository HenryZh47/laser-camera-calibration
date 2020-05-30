# LiDAR-Camera Calibration Toolbox

This repository contains toolbox and data management utilities for sensor calibration. All the extrinsic parameters of the sensors are calibrated to a VLP-16 LiDAR.

## Dependencies

1. Install gnupg in Python3 for rosbag compatibility. Use the following commandline input.

```bash
$ sudo pip3 install python-gnupg
```

2. Install MATLAB.

## Usage

### Data Recording

1. Get a checker board. Tested with the one in Mavlab with square size of 119mm\*119mm. If you are calibrating a thermal camera, leave the calibration board under sunlight or heat source for 5 minutes, then the thermal camera should be able to detect the checkerboard pattern.
2. Record the data into bag files. Make sure the checkerboard is within the FOV of the camera. Stand 3 to 6 meters away from the sensors. Each bag file should contain one static checkerboard pose. Record each pose bag for 10 seconds. Record 10-15 poses for each camera/LiDAR pair.

### Preparing for calibration

1. Setup a working directory with the following sub-folders: `bag`, `images`, `pcd`, `xyz`.
1. Put bag files to the `bag` sub-directory.
1. Extract images to `images` sub-directory with `util/save_one_image_from_bag.py`.
1. Extract `.xyz` point clouds to `xyz` sub-directory with `util/bag_to_xyz.sh` (You will need to change the path in this script).
1. Examine the extracted image data. Delete any (image, pointcloud) pair if the calibration target is invalid (out of FOV, large angle, etc).

### Camera intrinsic calibration

Use `/tool_box/intrinsic_calibration` to calibrate intrinsic paramters for each camera. [Here](http://www.vision.caltech.edu/bouguetj/calib_doc/) is the link to the tutorial of the toolbox.

Save the `Calib_Result.mat` file in the image folder.

Note1: This toolbox is relatively difficult to use, however the result format is necessary for LiDAR/camera extrinsics calibration.
Note2: The toolbox cannot properly save the result if there already exists files with name `Calib_Result.mat` in the matlab path.

### LiDAR camera extrinsic calibration

Use `/tool_box/extrinsic_calibration` to calibrate extrinsic paramters between one camera and VLP-16. [Here](http://www.cs.cmu.edu/~ranjith/lcct.html) is the link to the original toolbox and [here](https://github.com/zhixy/Laser-Camera-Calibration-Toolbox) is the link to the modified toolbox to improve LiDAR plane selection.

I also made some modifications to support sparse point cloud for VLP-16, but the procedure is basically the same. I suggest to read up both links above for more details.

Save the calibration result in the xyz folder.

### Checking the result

In xyz folder, run matlab script `project_to_cam_main.m` from `/util` to show the projected lidar points on the image.

See if the discontinuities of the lidar points are reasonable in the image (e.g. edge of the checker board).

