%% Lidar Localization with Unreal Engine Simulation
% This example shows how to develop and evaluate a lidar localization
% algorithm using synthetic lidar data from the Unreal Engine(R) simulation
% environment. 
%%

% Copyright 2020 The MathWorks, Inc.

%%
% Developing a localization algorithm and evaluating its performance in
% varying conditions is a challenging task. One of the biggest challenges
% is obtaining ground truth. Although you can capture ground truth using
% expensive, high-precision inertial navigation systems (INS), virtual
% simulation is a cost-effective alternative. The use of simulation enables
% testing under a variety of scenarios and sensor configurations. It also
% enables a rapid development iteration, and provides precise ground truth.

%%
% This example uses the Unreal Engine simulation environment from Epic
% Games(R) to develop and evaluate a lidar localization algorithm from a
% known initial pose in a parking scenario.

%% Set Up Scenario in Simulation Environment
% Parking a vehicle into a parking spot is a challenging maneuver that
% relies on accurate localization. Use the prebuilt
% <docid:driving_ref#mw_8ecb60fe-eb82-4f45-9629-a581750c6b69 Large Parking
% Lot> scene to create such a scenario. The
% <docid:driving_ug#mw_f780d508-aca7-46bb-b440-40ec9e6ff0af Select
% Waypoints for 3D Simulation> example describes how to interactively
% select a sequence of waypoints from a scene and how to generate a
% reference vehicle trajectory. This example uses a recorded reference
% trajectory obtained using the approach described in the linked example.
% First, visualize the reference path on a 2-D bird's-eye view of the
% scene.

% Load reference path
data = load('ReferencePathForward.mat');

refPosesX = data.ReferencePathForward.refPosesX; 
refPosesY = data.ReferencePathForward.refPosesY;
refPosesT = data.ReferencePathForward.refPosesT;

sceneName = 'LargeParkingLot';
hScene = figure;
helperShowSceneImage(sceneName);
hold on
scatter(refPosesX(:,2), refPosesY(:,2), [], 'filled', 'DisplayName', ...
    'Reference Path');
xlim([0 40])
ylim([-20 10])
legend
hold off

%% Record and Visualize Sensor Data
% Set up a simple model with a hatchback vehicle moving along the specified
% reference path by using the
% <docid:driving_ref#mw_32cd8e72-2d69-4c3e-98b0-5b918db383a4 Simulation 3D
% Vehicle with Ground Following> block. Mount a lidar on the roof center of
% a vehicle using the
% <docid:driving_ref#mw_f17d49dd-d3d9-40fb-b1cc-2e35dfdf5302 Simulation 3D
% Lidar> block. Record and visualize the sensor data. The recorded data is
% used to develop a localization algorithm.
close(hScene)

if ~ispc
    error("Unreal Engine Simulation is supported only on Microsoft" ...
        + char(174) + " Windows" + char(174) + ".");
end

% Open model
modelName = 'recordAndVisualize';
open_system(modelName);
snapnow;

% Run simulation
simOut = sim(modelName);

%%
% The recorded sensor data is returned in the |simOut| variable.

%% Develop Algorithm Using Recorded Data
% In this example, you develop an algorithm based on point cloud
% registration. Point cloud registration is a common localization technique
% that estimates the relative motion between two point clouds to derive
% localization data. Accumulating relative motion like this over long
% sequences can lead to drift, which can be corrected using loop closure
% detection and pose graph optimization, as shown in the
% <docid:vision_ug#mw_5ec6a491-44b6-405f-8c6d-74fc0f28334f Build a Map from Lidar Data Using SLAM>
% example. Since this example uses a short
% reference path, loop closure detection is omitted.

%%
% Extract the lidar sensor data and ground truth location and orientation
% provided by the
% <docid:driving_ref#mw_f17d49dd-d3d9-40fb-b1cc-2e35dfdf5302 Simulation 3D
% Lidar> block. The ground truth location and orientation are provided in
% the world (scene) coordinate system. Extract the known initial pose from
% the ground truth data by using the |helperPoseToRigidTransform| function.

close_system(modelName);

% Extract lidar sensor data
ptCloudArr = helperGetPointClouds(simOut);

% Extract ground truth
[lidarLocation, lidarOrientation, simTimes] = helperGetLocalizerGroundTruth(simOut);

% Compute initial pose
initPose = [lidarLocation(1, :) lidarOrientation(1, :)];
initTform = helperPoseToRigidTransform(initPose);

%%
% Develop a lidar localization algorithm by using the extracted sensor
% data. Use a |<docid:vision_ref#mw_c6968c3b-08dd-4aaa-94e7-b6c0e0185f8d
% pcviewset>| object to process and store odometry data. |pcviewset|
% organizes odometry data into a set of views, and the associated
% connections between views, where:
% 
% * Each view has an absolute pose describing the rigid transformation to
% some fixed reference frame.
% * Each connection has a relative pose describing the rigid transformation
% between the two connecting views.
% 
% The localization estimate is maintained in the form of the absolute poses
% for each view with respect to the scene reference frame.

%%
% Use a |<docid:vision_ref#busm5az-7 pcplayer>| object to display streaming
% point cloud data in the loop as it is registered. Transform the viewing
% angle to a top view. The orange cuboid and path show the localization
% position estimated by the algorithm. The green path shows the ground
% truth.

% Create a view set
vSet = pcviewset;

absPose = initTform;
relPose = rigid3d;
viewId  = 1;

% Define rigid transformation between the lidar sensor mounting position
% and the vehicle reference point. 
lidarToVehicleTform = helperPoseToRigidTransform(single([0 0 -1.57 0 0 0]));

% Process the point cloud frame
ptCloud = helperProcessPointCloud(ptCloudArr(1));

% Initialize accumulated point cloud map
ptCloudAccum = pctransform(ptCloud, absPose);

% Add first view to the view set
vSet = addView(vSet, viewId, absPose, 'PointCloud', ptCloud);

% Configure display
xlimits = [  0  50];
ylimits = [-25  10];
zlimits = [-30  30];
player = pcplayer(xlimits, ylimits, zlimits);
estimatePathHandle = [];
truthPathHandle    = [];

% Specify vehicle dimensions
centerToFront = 1.104;
centerToRear  = 1.343;
frontOverhang = 0.828;
rearOverhang  = 0.589;
vehicleWidth  = 1.653;
vehicleHeight = 1.513;
vehicleLength = centerToFront + centerToRear + frontOverhang + rearOverhang;
hatchbackDims = vehicleDimensions(vehicleLength,vehicleWidth,vehicleHeight, ...
'FrontOverhang',frontOverhang,'RearOverhang',rearOverhang);

vehicleDims   = [hatchbackDims.Length, hatchbackDims.Width, hatchbackDims.Height];
vehicleColor  = [0.85 0.325 0.098];

% Initialize parameters
skipFrames    = 5;      % Number of frames to skip to accumulate sufficient motion
prevViewId    = viewId;
prevPtCloud   = ptCloud;

% Loop over lidar sensor frames and localize
for viewId = 6 : skipFrames : numel(ptCloudArr)
    % Process frame
    ptCloud = helperProcessPointCloud(ptCloudArr(viewId));
    
    % Register current frame to previous frame
    relPose = pcregistericp(ptCloud, prevPtCloud, 'MaxIterations', 40, ...
        'Metric', 'pointToPlane');
    
    % Since motion is restricted to a 2-D plane, discard motion along Z to
    % prevent accumulation of noise.
    relPose.Translation(3) = 0;
    
    % Update absolute pose
    height = absPose.Translation(3);
    absPose = rigid3d( relPose.T * absPose.T );
    absPose.Translation(3) = height;
    
    % Add new view and connection to previous view
    vSet = addView(vSet, viewId, absPose, 'PointCloud', ptCloud);
    vSet = addConnection(vSet, prevViewId, viewId, relPose);
    
    % Accumulated point cloud map
    ptCloudAccum = pccat([ptCloudAccum, pctransform(ptCloud, absPose)]);
    
    % Compute ground truth and estimate position
    localizationEstimatePos = absPose.Translation;
    localizationTruthPos    = lidarLocation(viewId, :);
    
    % Update accumulated point cloud map
    view(player, ptCloudAccum);
    
    % Set viewing angle to top view
    view(player.Axes, 2);
    
    % Convert current absolute pose of sensor to vehicle frame 
    absVehiclePose = rigid3d( lidarToVehicleTform.T * absPose.T );
    
    % Draw vehicle at current absolute pose
    helperDrawVehicle(player.Axes, absVehiclePose, vehicleDims, 'Color', vehicleColor);
    
    % Draw localization estimate and ground truth points
    helperDrawLocalization(player.Axes, ...
        localizationEstimatePos, estimatePathHandle, vehicleColor, ...
        localizationTruthPos, truthPathHandle, [0 1 0]);
    
    prevPtCloud = ptCloud;
    prevViewId  = viewId;
end

%%
% Zoom in to the tail of the trajectory to examine the localization
% estimate compared to the ground truth.

xlim(player.Axes, [0 15]);
ylim(player.Axes, [-15 0]);
zlim(player.Axes, [0 15]);

snapnow;

% Close player
hide(player);

%%
% A useful outcome of a localization algorithm based on point cloud
% registration is a map of the traversed environment. You can obtain this
% map by combining all the point clouds to a common reference frame. The
% |<docid:vision_ref#mw_c2ec5b99-73ec-4ff0-9581-4d0578ec2f12 pccat>|
% function is used in each iteration of the loop above, along with
% |<docid:vision_ref#bupmtw7-1 pctransform>|, to incrementally combine the
% registered point clouds. Alternatively, you can use the
% |<docid:vision_ref#mw_943aeeec-bf72-49f6-bffd-9a2295e6b6f4 pcalign>|
% function to align all point clouds to the common reference frame in one
% shot at the end.

%%
% Superimpose the point cloud map on the top-view image of the scene to
% visually examine how closely it resembles features in the scene.

hMapOnScene = helperSuperimposeMapOnSceneImage(sceneName, ptCloudAccum);

snapnow;

% Close the figure
close(hMapOnScene);

%%
% The localization algorithm described above is encapsulated in the
% |helperLidarRegistrationLocalizer| helper class. This class can be used
% as a framework to develop a localization pipeline using point cloud
% registration. 
%
% * Use the |'ProcessFcnHandle'| and |'ProcessFcnArguments'| name-value
% pair arguments to configure how point clouds are processed prior to
% registration. 
% * Use the |'RegisterFcnHandle'| and |'RegisterFcnArguments'| name-value
% pair arguments to configure how point clouds are registered.

%% Evaluate Localization Accuracy
% To quantify the efficacy of localization, measure the deviation in
% translation and rotation estimates compared to ground truth. Since the
% vehicle is moving on flat ground, this example is concerned only with
% motion in the _X-Y_ plane.

hFigMetrics = helperDisplayMetrics(vSet, lidarLocation, lidarOrientation, simTimes);

%% Simulate in the Loop
% Although metrics like deviation in translation and rotation estimates are
% necessary, the performance of a localization system can have downstream
% impacts. For example, changes to the accuracy or performance of a
% localization system can affect the vehicle controller, necessitating the
% retuning of controller gains. Therefore, it is crucial to have a
% closed-loop verification framework that incorporates downstream
% components. The |localizeAndControlUsingLidar| model demonstrates this
% framework by incorporating a localization algorithm, vehicle controller
% and suitable vehicle model.

%%
% The model has these main components:
%
% * The Localize block is a MATLAB Function block that encapsulates the
% localization algorithm - implemented using the
% |helperLidarRegistrationLocalizer| class. This block takes the lidar
% point cloud generated by the
% <docid:driving_ref#mw_f17d49dd-d3d9-40fb-b1cc-2e35dfdf5302 Simulation 3D
% Lidar> block and the initial known pose as inputs and produces a
% localization estimate. The estimate is returned as $(x, y, \theta)$,
% which represents the 2-D pose of the lidar in the map reference frame.
% * The Plan subsystem loads a preplanned trajectory from the workspace
% using the |refPoses|, |directions|, |curvatures| and |velocities|
% workspace variables. The *Path Smoother Spline* block was used to compute
% the |refPoses|, |directions| and |curvatures| variables. The *Velocity
% Profiler* block computed the |velocities| variable.
% * The Helper Path Analyzer block uses the reference trajectory and the
% current pose to feed the appropriate reference signal to the vehicle
% controller.
% * The Vehicle Controller subsystem controls the steering and velocity of
% the vehicle by using a lateral and longitudinal controller to produce a
% steering and acceleration or deceleration command. The *Lateral
% Controller Stanley* and *Longitudinal Controller Stanley* blocks are used
% to implement this. These commands are fed to a vehicle model to
% simulate the dynamics of the vehicle in the simulation environment using
% the *Vehicle Body 3DOF* block.

close(hFigMetrics);

% Load workspace variables for preplanned trajectory
refPoses   = data.ReferencePathForward.Trajectory.refPoses;
directions = data.ReferencePathForward.Trajectory.directions;
curvatures = data.ReferencePathForward.Trajectory.curvatures;
velocities = data.ReferencePathForward.Trajectory.velocities;
startPose  = refPoses(1, :);

% Open model
modelName = 'localizeAndControlUsingLidar';
open_system(modelName);
snapnow;

% Run simulation
sim(modelName);

close_system(modelName);

%%
% With this setup, it is possible to rapidly iterate over different
% scenarios, sensor configurations, or reference trajectories and refine
% the localization algorithm before moving to real-world testing.
% 
% * To select a different scenario, use the
% <docid:driving_ref#mw_96a521fd-316f-497b-bc01-b2c5f4083563 Simulation 3D
% Scene Configuration> block. Choose from the existing prebuilt scenes or
% create a custom scene in the Unreal(R) Editor.
% * To create a different reference trajectory, use the
% |helperSelectSceneWaypoints| tool, as shown in the
% <docid:driving_ug#mw_f780d508-aca7-46bb-b440-40ec9e6ff0af Select
% Waypoints for 3D Simulation> example.
% * To alter the sensor configuration use the
% <docid:driving_ref#mw_f17d49dd-d3d9-40fb-b1cc-2e35dfdf5302 Simulation 3D
% Lidar> block. The *Mounting* tab provides options for specifying
% different sensor mounting placements. The *Parameters* tab provides
% options for modifying sensor parameters such as detection range, field of
% view, and resolution.



%% Supporting Functions
%%%
% *|helperGetPointClouds|* Extract an array of |pointCloud| objects that
% contain lidar sensor data.
function ptCloudArr = helperGetPointClouds(simOut)

% Extract signal
ptCloudData = simOut.ptCloudData.signals.values;

% Create a pointCloud array
ptCloudArr = pointCloud(ptCloudData(:,:,:,3)); % Ignore first 2 frames
for n = 4 : size(ptCloudData,4)
    ptCloudArr(end+1) = pointCloud(ptCloudData(:,:,:,n));  %#ok<AGROW>
end
end

%%%
% *|helperGetLocalizerGroundTruth|* Extract ground truth location and
% orientation.
function [lidarLocation, lidarOrientation, simTimes] = helperGetLocalizerGroundTruth(simOut)

lidarLocation    = squeeze(simOut.lidarLocation.signals.values)';
lidarOrientation = squeeze(simOut.lidarOrientation.signals.values)';
simTimes         = simOut.lidarLocation.time;

% Ignore first 2 frames
lidarLocation(1:2, :)    = [];
lidarOrientation(1:2, :) = [];
simTimes(1:2, :)         = [];
end

%%%
% *|helperDrawLocalization|* Draw localization estimate and ground truth on
% axes.
function [estHandle,truthHandle] = helperDrawLocalization(axesHandle, ...
    est, estHandle, estColor, truth, truthHandle, truthColor)

% Create scatter objects and adjust legend
if isempty(estHandle) || isempty(truthHandle)
    markerSize = 6;
    hold(axesHandle, 'on');
    estHandle   = scatter3(axesHandle, NaN, NaN, NaN, markerSize, estColor, 'filled');
    truthHandle = scatter3(axesHandle, NaN, NaN, NaN, markerSize, truthColor, 'filled');
    %legend(axesHandle, {'Points', 'Estimate', 'Truth'}, ...
    %    'Color', [1 1 1], 'Location', 'northeast');
    hold(axesHandle, 'off');
end

estHandle.XData(end+1) = est(1);
estHandle.YData(end+1) = est(2);
estHandle.ZData(end+1) = est(3);

truthHandle.XData(end+1) = truth(1);
truthHandle.YData(end+1) = truth(2);
truthHandle.ZData(end+1) = truth(3);
end

%%%
% *|helperSuperimposeMapOnSceneImage|* Superimpose point cloud map on scene
% image
function hFig = helperSuperimposeMapOnSceneImage(sceneName, ptCloudAccum)

hFig = figure('Name', 'Point Cloud Map');
hIm = helperShowSceneImage(sceneName);

hold(hIm.Parent, 'on')
pcshow(ptCloudAccum);
hold(hIm.Parent, 'off')

xlim(hIm.Parent, [-10 50]);
ylim(hIm.Parent, [-30 20]);
end

%%%
% *|helperDisplayMetrics|* Display metrics to assess quality of
% localization.
function hFig = helperDisplayMetrics(vSet, lidarLocation, lidarOrientation, simTimes)

absPoses = vSet.Views.AbsolutePose;
translationEstimates = vertcat(absPoses.Translation);
rotationEstimates    = pagetranspose(cat(3, absPoses.Rotation));

viewIds = vSet.Views.ViewId;
viewTimes = simTimes(viewIds);

xEst   = translationEstimates(:, 1);
yEst   = translationEstimates(:, 2);
yawEst = euler(quaternion(rotationEstimates, 'rotmat', 'point'), 'ZYX', 'point');
yawEst = yawEst(:, 1);

xTruth   = lidarLocation(viewIds, 1);
yTruth   = lidarLocation(viewIds, 2);
yawTruth = lidarOrientation(viewIds, 3);

xDeviation   = abs(xEst - xTruth);
yDeviation   = abs(yEst - yTruth);
yawDeviation = abs(helperWrapToPi(yawTruth - yawEst));

hFig = figure('Name', 'Metrics - Absolute Deviation');
subplot(3,1,1)
plot(viewTimes, xDeviation, 'LineWidth', 2);
ylim([0 1])
grid on
title('X')
xlabel('Time (s)')
ylabel('Deviation (m)')

subplot(3,1,2)
plot(viewTimes, yDeviation, 'LineWidth', 2);
ylim([0 1])
grid on
title('Y')
xlabel('Time (s)')
ylabel('Deviation (m)')

subplot(3,1,3)
plot(viewTimes, yawDeviation, 'LineWidth', 2);
ylim([0 pi/20])
grid on
title('Yaw')
xlabel('Time (s)')
ylabel('Deviation (rad)')
end

%%%
% *|helperWrapToPi|* Wrap angles to be in the range $[-\pi, \pi]$.
function angle = helperWrapToPi(angle)

idx = (angle < -pi) | (angle > pi);
angle(idx) = helperWrapTo2Pi(angle(idx) + pi) - pi;
end

%%%
% *|helperWrapTo2Pi|* Wrap angles to be in the range $[-2\pi, 2\pi]$.
function angle = helperWrapTo2Pi(angle)

pos = (angle>0);
angle = mod(angle, 2*pi);
angle(angle==0 & pos) = 2*pi;
end