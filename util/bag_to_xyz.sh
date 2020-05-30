#!/bin/sh

# Shell script that takes one frame of LiDAR scan out of the bag file and save it as a .xyz file
# Source bag file directory: /home/henryzh47/Projects/flir_thermal_camera_calibration/data/lidar_thermal_extrinsics/2019-05-31-bag
# Target xyz file directory: /home/henryzh47/Projects/flir_thermal_camera_calibration/data/lidar_thermal_extrinsics/2019-05-31-xyz

DATA_DIR=/media/henryzh47/henry-logging-2/rosbags/2020-05-21-ds-ueye-calibration/cam-lidar
BAG_DIR=$DATA_DIR/bag
PCD_DIR=$DATA_DIR/pcd
XYZ_DIR=$DATA_DIR/xyz
UTIL_DIR=`pwd`
PC_TOPIC=/uav3/velodyne_points

# Clear PCD and XYZ dir
rm $PCD_DIR/*.pcd
rm $XYZ_DIR/*.xyz

# Loop through all the files in the bag directory
xyz_index=1
for bag in $BAG_DIR/*.bag
do 
	echo "processing $bag"
	rosrun pcl_ros bag_to_pcd $bag $PC_TOPIC $PCD_DIR 
	echo "PCD files saved for bag: $bag"
	
	# pick the first PCD file in the dir
	pcd=$(ls $PCD_DIR | head -n 1)
	echo "picked $pcd"
	cd $PCD_DIR
	mv $pcd source.pcd

	# run the MATLAB script to trim the point cloud and save to xyz dir
	sleep 1
	cd $UTIL_DIR
	matlab -nodisplay -batch 'pcd_trim("/media/henryzh47/henry-logging-2/rosbags/2020-05-21-ds-ueye-calibration/cam-lidar/pcd/source.pcd", "/media/henryzh47/henry-logging-2/rosbags/2020-05-21-ds-ueye-calibration/cam-lidar/xyz/target.xyz")'
	cd $XYZ_DIR
	mv ./target.xyz target_scan$xyz_index.xyz
	xyz_index=$(($xyz_index+1))

	# delete all the pcd for this bag
	cd $PCD_DIR
	rm *
done
