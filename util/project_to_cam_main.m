%% Project Velodyne points to camera
CAMERA_INFO_PATH = './../images/Calib_Results.mat';

PC_PATH = './target_scan10.xyz';
IMG_PATH = './../images/image_rect10.bmp';

%%  Load intrinsics and extrinsics parameters
% Intrinsics
camera_intrinsics = load(CAMERA_INFO_PATH);
K = camera_intrinsics.KK;
% Extrinsics
% realsense_extrinsic_calib_2;

% Extrinsics from CAD
%t = [.08315; -0.0495; -.055846];
%R = [0, -1, 0; 0, 0, -1; 1, 0, 0];

T = [R, t];

%% Load the LiDAR points
points = load(PC_PATH);
points = points';
num_points = size(points, 2);
% make points in homogeneous coordinate
points = [points; ones(1, num_points)];

%% Apply transforms to the points to image pixel location
points_transformed = K * T * points;
points_transformed = points_transformed ./ repmat(points_transformed(3,:), 3, 1);
size(points_transformed)
visible_points_index = (points_transformed(1,:)>0 & ...
                        points_transformed(1,:)<1024 & ...
                        points_transformed(2,:)>0 & ...
                        points_transformed(2,:)<770);
points_visible = points_transformed(:,visible_points_index);
points_visible_uv = uint32(points_visible(1:2, :));


%% Read-in the image and overlay the lidar points
img = imread(IMG_PATH);
img_with_points = insertMarker(img, points_visible_uv');
imshow(img_with_points);


