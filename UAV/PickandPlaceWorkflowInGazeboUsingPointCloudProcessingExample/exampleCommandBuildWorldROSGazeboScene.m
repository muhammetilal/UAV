function exampleCommandBuildWorldROSGazeboScene(coordinator)

% CommandBuildWorld 
% This function uses input from the depth camera attached to the robot in
% order to build the robot planning scene. The resulting collision meshes
% will be used by RRT planner to generate collision-free paths when picking
% up parts. The following steps are implemented:
% Step 1: Move to scanning positions and capture point cloud data and camera poses
% Step 2: Transform point clouds from camera to world frame 
% Step 3: Merge transformed pointclouds to single point cloud
% Step 4: Segment point cloud
% Step 5: Create collision meshes for all segmented point clouds
% Step 6: Add collision meshes to the planning "world" to be used by RRT
% planner

% Copyright 2020 The MathWorks, Inc.
 
    %% Step 1: Move to scanning positions and capture point cloud data and camera poses
    tftree = rostf;
    camera_quaternions = cell(numel(coordinator.ScanPoses),1); 
    camera_translations = cell(numel(coordinator.ScanPoses),1); 
    pointclouds = cell(numel(coordinator.ScanPoses),1);
    for scanIter = 1: numel(coordinator.ScanPoses)
        % Move to next scanning pose
        disp(['Moving to scanning pose ' num2str(scanIter)]);
        exampleCommandMoveToTaskConfigROSGazeboScene(coordinator, ...
        coordinator.ScanPoses{scanIter});
        pause(2);
        % Capture point cloud
        disp(['Capturing point cloud ' num2str(scanIter)]);
        pointCloudSub = rossubscriber('/camera/depth/points');
        pointclouds{scanIter} = receive(pointCloudSub);
        % Get camera pose
        disp(['Getting camera pose ' num2str(scanIter)]);
        camera_transf = getTransform(tftree, 'world', 'camera_link');        
        camera_transl = camera_transf.Transform.Translation;
        camera_rotation = camera_transf.Transform.Rotation;
        camera_quaternions{scanIter} = [camera_rotation.W, camera_rotation.X,...
            camera_rotation.Y,camera_rotation.Z];
        camera_translations{scanIter} = [camera_transl.X,...
            camera_transl.Y,camera_transl.Z];
    end
    %% Steps 2 and 3: Transform point clouds to world frame and merge to single point cloud
    pointclouds = pointclouds';
    for i=1:size(pointclouds,2)
        % Extract xyz locations of point cloud
        xyz = readXYZ(pointclouds{1,i});
        
        % Remove NaNs
        xyzLess = double(rmmissing(xyz));

        % Create point cloud object
        ptCloud = pointCloud(xyzLess);

        % Create affinite transformation from camera quaternions and
        % translations
        quat = camera_quaternions{i};
        rotm = quat2rotm(quat);
        fixedRotation = eul2rotm([0 pi 0],"XYZ"); % fixed rotation between gazebo camera and urdf camera link
        rotm = rotm*fixedRotation' ;
        translVect = camera_translations{i};
        tform = rigid3d(rotm,[translVect(1),translVect(2),translVect(3)]);
     
        % Transform point cloud to world frame
        ptCloudWorld = pctransform(ptCloud,tform);
        
        % Merge with other point clouds
        if i == 1
            pcMerged = ptCloudWorld;
        else
            mergeSize = 0.015;
            pcMerged = pcmerge(pcMerged, ptCloudWorld, mergeSize);
        end
    end
    pause(3);
    
    % Remove points below table height
    indxPlane = find(pcMerged.Location(:,3) > -0.09 & pcMerged.Location(:,3) < -0.07);
    plane = select(pcMerged,indxPlane); % table   
    indx = find(pcMerged.Location(:,3) > -0.08);
    pcMergedTop = select(pcMerged,indx);
    coordinator.MergedPointCloud = pcMergedTop;

    % Initialize list of obstacles with table
    tableMesh = collisionMesh(plane.Location);
    tablePointCloud = plane;
    hold on
    [~,patchObj] = show(tableMesh);
    patchObj.LineStyle = 'none';
    patches = {patchObj}; 
    obstacles = {tableMesh};
    segments = {tablePointCloud};
    drawnow;
    pause(1);

    %% Step 4: Segment point cloud
    [labels, numClusters] = pcsegdist(pcMergedTop,0.05);
    
    %% Step 5: Create collision meshes for segmented point clouds
    for i=1:numClusters
        labelIdx = find(labels==i);
        obstacle = select(pcMergedTop,labelIdx);
        if obstacle.Count > 100
            obstacleMesh = collisionMesh(obstacle.Location);
            hold on
            [~,patchObj] =  show(obstacleMesh);
            patchObj.FaceColor = rand(1,3);
            patchObj.LineStyle = 'none';
            obstacles{end+1} = copy(obstacleMesh);
            segments{end+1} = copy(obstacle);
            patches{end+1} =  patchObj; 
            drawnow;
            pause(1);
        end        
    end
    %% Step 6: Add collision meshes to the planning "world"
    coordinator.World = obstacles;
    % Store point cloud segments
    coordinator.PointCloudSegments = segments;
    % Store patch handles of World meshes for removing when a part is
    % picked
    coordinator.WorldPatchHandles = patches;
end

