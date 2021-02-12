function [] = pcd_trim(CLOUD_PATH, OUTPUT_PATH)

    %% PCD trim for lidar-camera calibration
    %% parameters
    max_dist = 5;
    min_dist = 0.25;

    %% Read in the PCD file
    ptCloud = pcread(CLOUD_PATH);
    %pcshow(ptCloud);

    %% Discard points that are over the target range
    xyz_locations = ptCloud.Location;
    num_points = length(xyz_locations);
    new_xyz_locations = zeros(1,3);
    new_index = 1;

    for i = 1:num_points
        xyz = xyz_locations(i,:);
        dist_to_center = sqrt(sum(xyz.^2));
        if (dist_to_center < max_dist && dist_to_center > min_dist)
            new_xyz_locations(new_index,:) = xyz;
            new_index = new_index + 1;
        end
    end

    new_ptCloud = pointCloud(new_xyz_locations);
    %pcshow(new_ptCloud);

    %% Save the result as a xyz file for calibration pipeline
    dlmwrite(OUTPUT_PATH, new_xyz_locations, 'delimiter', ' ');

end

