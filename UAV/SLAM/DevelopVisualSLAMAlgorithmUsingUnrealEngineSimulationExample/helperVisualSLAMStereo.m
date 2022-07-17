function [mapPlot, optimizedPoses, addedFramesIdx] = helperVisualSLAMStereo(imdsLeft, imdsRight, intrinsics, baseline)
%helperVisualSLAMStereo Evaluate the performance of a stereo visual SLAM algorithm
%   The implementation details of the stereo visual SLAM algorithm can be 
%   found in the Stereo Visual Simultaneous Localization and Mapping example.
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.

%   Copyright 2020-2021 The MathWorks, Inc.

% Set random seed for reproducibility
rng(0);

%% Map Initialization

% Create a stereoParameters object to store the stereo camera parameters.
cameraParam     = cameraParameters('IntrinsicMatrix', intrinsics.IntrinsicMatrix);
stereoParams    = stereoParameters(cameraParam, cameraParam, eye(3), [-baseline, 0 0]);

% Read the first pair of stereo images
currFrameIdx   = 1;
currILeft      = readimage(imdsLeft, currFrameIdx);
currIRight     = readimage(imdsRight, currFrameIdx);

% In this example, the images are already undistorted. In a general
% workflow, uncomment the following code to undistort the images.
% currILeft  = undistortImage(currILeft, intrinsics);
% currIRight = undistortImage(currIRight, intrinsics);

% Rectify the stereo images
[currILeft, currIRight] = rectifyStereoImages(currILeft, currIRight, stereoParams, 'OutputView','full');

% Detect and extract ORB features from the rectified stereo images
scaleFactor = 1.2;
numLevels   = 8;
[currFeaturesLeft,  currPointsLeft]   = helperDetectAndExtractFeatures(currILeft, scaleFactor, numLevels); 
[currFeaturesRight, currPointsRight]  = helperDetectAndExtractFeatures(currIRight, scaleFactor, numLevels);

% Match feature points between the stereo images and get the 3-D world positions 
maxDisparity = 48;
initialPose = rigid3d;
[xyzPoints, matchedPairs] = helperReconstructFromStereo(currILeft, currIRight, ...
    currFeaturesLeft, currFeaturesRight, currPointsLeft, currPointsRight, stereoParams, initialPose, maxDisparity);

%% Data Management and Visualization
% Create an empty imageviewset object to store key frames
vSetKeyFrames = imageviewset;

% Create an empty worldpointset object to store 3-D map points
mapPointSet   = worldpointset;

% Create a helperViewDirectionAndDepth object to store view direction and depth 
directionAndDepth = helperViewDirectionAndDepth(size(xyzPoints, 1));

% Add the first key frame
currKeyFrameId = 1;
vSetKeyFrames = addView(vSetKeyFrames, currKeyFrameId, initialPose, 'Points', currPointsLeft,...
    'Features', currFeaturesLeft.Features);

% Add 3-D map points
[mapPointSet, stereoMapPointsIdx] = addWorldPoints(mapPointSet, xyzPoints);

% Add observations of the map points
mapPointSet = addCorrespondences(mapPointSet, currKeyFrameId, stereoMapPointsIdx, matchedPairs(:, 1));

% Visualize matched features in the first key frame
featurePlot = helperVisualizeMatchedFeaturesStereo(currILeft, currIRight, currPointsLeft, ...
    currPointsRight, matchedPairs);

% Visualize initial map points and camera trajectory
mapPlot     = helperVisualizeSceneAndTrajectoryStereo(vSetKeyFrames, mapPointSet);

% Show legend
showLegend(mapPlot);

%% Initialize Place Recognition Database

% Load the bag of features data created offline
bofData         = load('bagOfFeaturesDataParkingLot.mat');

% Initialize the place recognition database
loopDatabase    = invertedImageIndex(bofData.bof, "SaveFeatureLocations", false);

% Add features of the first key frame to the database
addImageFeatures(loopDatabase, currFeaturesLeft, currKeyFrameId);

%% Tracking
% The tracking process is performed using every frame and determines when to 
% insert a new key frame. To simplify this example, we will terminate the tracking 
% process once a loop closure is found.

% ViewId of the last key frame
lastKeyFrameId    = currKeyFrameId;

% Index of the last key frame in the input image sequence
lastKeyFrameIdx   = currFrameIdx; 

% Indices of all the key frames in the input image sequence
addedFramesIdx    = lastKeyFrameIdx;

currFrameIdx      = 2;
isLoopClosed      = false;

% Main loop
while ~isLoopClosed && currFrameIdx < numel(imdsLeft.Files)

    currILeft  = readimage(imdsLeft, currFrameIdx);
    currIRight = readimage(imdsRight, currFrameIdx);
    [currILeft, currIRight] = rectifyStereoImages(currILeft, currIRight, stereoParams, 'OutputView','full');

    [currFeaturesLeft, currPointsLeft]    = helperDetectAndExtractFeatures(currILeft, scaleFactor, numLevels);
    [currFeaturesRight, currPointsRight]  = helperDetectAndExtractFeatures(currIRight, scaleFactor, numLevels);

    % Track the last key frame
    % trackedMapPointsIdx:  Indices of the map points observed in the current frame
    % trackedFeatureIdx:    Indices of the corresponding feature points in the current frame
    [currPose, trackedMapPointsIdx, trackedFeatureIdx] = helperTrackLastKeyFrame(mapPointSet, ...
        vSetKeyFrames.Views, currFeaturesLeft, currPointsLeft, lastKeyFrameId, intrinsics, scaleFactor);
    
    if isempty(currPose) || numel(trackedMapPointsIdx) < 30
        currFrameIdx = currFrameIdx + 1;
        continue
    end
    
    % Track the local map
    % refKeyFrameId:      ViewId of the reference key frame that has the most 
    %                     co-visible map points with the current frame
    % localKeyFrameIds:   ViewId of the connected key frames of the current frame
    if currKeyFrameId == 1
        refKeyFrameId    = 1;
        localKeyFrameIds = 1;
    else
        [refKeyFrameId, localKeyFrameIds, currPose, trackedMapPointsIdx, trackedFeatureIdx] = ...
            helperTrackLocalMap(mapPointSet, directionAndDepth, vSetKeyFrames, trackedMapPointsIdx, ...
            trackedFeatureIdx, currPose, currFeaturesLeft, currPointsLeft, intrinsics, scaleFactor, numLevels);
    end
    
    % Match feature points between the stereo images and get the 3-D world positions
    [xyzPoints, matchedPairs] = helperReconstructFromStereo(currILeft, currIRight, currFeaturesLeft, ...
        currFeaturesRight, currPointsLeft, currPointsRight, stereoParams, currPose, maxDisparity);
    
    [untrackedFeatureIdx, ia] = setdiff(matchedPairs(:, 1), trackedFeatureIdx);
    xyzPoints = xyzPoints(ia, :);
    
    % Check if the current frame is a key frame
    isKeyFrame = helperIsKeyFrame(mapPointSet, refKeyFrameId, lastKeyFrameIdx, ...
        currFrameIdx, trackedMapPointsIdx);

    % Visualize matched features in the stereo image
    updatePlot(featurePlot, currILeft, currIRight, currPointsLeft, currPointsRight, trackedFeatureIdx, matchedPairs);
    
    if ~isKeyFrame
        currFrameIdx = currFrameIdx + 1;
        continue
    end
    
    % Update current key frame ID
    currKeyFrameId  = currKeyFrameId + 1;

%% Local Mapping

    % Add the new key frame    
    [mapPointSet, vSetKeyFrames] = helperAddNewKeyFrame(mapPointSet, vSetKeyFrames, ...
        currPose, currFeaturesLeft, currPointsLeft, trackedMapPointsIdx, trackedFeatureIdx, localKeyFrameIds);
        
    % Remove outlier map points that are observed in fewer than 3 key frames
    if currKeyFrameId == 2
        triangulatedMapPointsIdx = [];
    end
    
    [mapPointSet, directionAndDepth, trackedMapPointsIdx] = ...
        helperCullRecentMapPoints(mapPointSet, directionAndDepth, trackedMapPointsIdx, triangulatedMapPointsIdx, ...
        stereoMapPointsIdx);
    
    % Add new map points computed from disparity 
    [mapPointSet, stereoMapPointsIdx] = addWorldPoints(mapPointSet, xyzPoints);
    mapPointSet = addCorrespondences(mapPointSet, currKeyFrameId, stereoMapPointsIdx, ...
        untrackedFeatureIdx);
    
    % Create new map points by triangulation
    minNumMatches = 20;
    minParallax   = 0.35;
    [mapPointSet, vSetKeyFrames, triangulatedMapPointsIdx, stereoMapPointsIdx] = helperCreateNewMapPointsStereo( ...
        mapPointSet, vSetKeyFrames, currKeyFrameId, intrinsics, scaleFactor, minNumMatches, minParallax, ...
        untrackedFeatureIdx, stereoMapPointsIdx);
    
    % Update view direction and depth
    directionAndDepth = update(directionAndDepth, mapPointSet, vSetKeyFrames.Views, ...
        [trackedMapPointsIdx; triangulatedMapPointsIdx; stereoMapPointsIdx], true);
    
    % Local bundle adjustment
    [mapPointSet, directionAndDepth, vSetKeyFrames, triangulatedMapPointsIdx, stereoMapPointsIdx] = ...
        helperLocalBundleAdjustmentStereo(mapPointSet, directionAndDepth, vSetKeyFrames, ...
        currKeyFrameId, intrinsics, triangulatedMapPointsIdx, stereoMapPointsIdx); 
    
    % Visualize 3-D world points and camera trajectory
    updatePlot(mapPlot, vSetKeyFrames, mapPointSet);
    
%% Loop Closure
    % Check loop closure after some key frames have been created    
    if currKeyFrameId > 100
        
        % Minimum number of feature matches of loop edges
        loopEdgeNumMatches = 50;
        
        % Detect possible loop closure key frame candidates
        [isDetected, validLoopCandidates] = helperCheckLoopClosure(vSetKeyFrames, currKeyFrameId, ...
            loopDatabase, currILeft, loopEdgeNumMatches);
        
        isTooCloseView = currKeyFrameId - max(validLoopCandidates) < 70;
        if isDetected && ~isTooCloseView
            % Add loop closure connections
            isStereo = true;
            [isLoopClosed, mapPointSet, vSetKeyFrames] = helperAddLoopConnections(...
                mapPointSet, vSetKeyFrames, validLoopCandidates, currKeyFrameId, ...
                currFeaturesLeft, currPointsLeft, loopEdgeNumMatches, isStereo);
        end
    end
    
    % If no loop closure is detected, add current features into the database
    if ~isLoopClosed
        addImageFeatures(loopDatabase,  currFeaturesLeft, currKeyFrameId);
    end
    
    % Update IDs and indices
    lastKeyFrameId  = currKeyFrameId;
    lastKeyFrameIdx = currFrameIdx;
    addedFramesIdx  = [addedFramesIdx; currFrameIdx]; 
    currFrameIdx    = currFrameIdx + 1;
end % End of main loop

if ~isLoopClosed
    disp('Loop closure cannot be found');
    optimizedPoses = [];
    return
end

% Create a pose graph from the key frames set
G = createPoseGraph(vSetKeyFrames);

% Optimize the pose graph
optimG = optimizePoseGraph(G, 'g2o-levenberg-marquardt');
optimizedPoses = optimG.Nodes;

% Update the view poses
vSetKeyFramesOptim = updateView(vSetKeyFrames, optimizedPoses);

% Update map points after optimizing the poses
mapPointSet = helperUpdateGlobalMap(mapPointSet, directionAndDepth, ...
    vSetKeyFrames, vSetKeyFramesOptim);

updatePlot(mapPlot, vSetKeyFrames, mapPointSet);

% Plot the optimized camera trajectory
plotOptimizedTrajectory(mapPlot, optimizedPoses)

% Update legend
showLegend(mapPlot);
end

%--------------------------------------------------------------------------
function [features, validPoints] = helperDetectAndExtractFeatures(Irgb, ...
    scaleFactor, numLevels, varargin)
%helperDetectAndExtractFeatures detect and extract features

numPoints   = 1000;

% In this example, the images are already undistorted. In a general
% workflow, uncomment the following code to undistort the images.
%
% if nargin > 3
%     intrinsics = varargin{1};
% end
% Irgb  = undistortImage(Irgb, intrinsics);

% Detect ORB features
Igray  = rgb2gray(Irgb);

points = detectORBFeatures(Igray, 'ScaleFactor', scaleFactor, 'NumLevels', numLevels);

% Select a subset of features, uniformly distributed throughout the image
points = selectUniform(points, numPoints, size(Igray, 1:2));

% Extract features
[features, validPoints] = extractFeatures(Igray, points);
end

%--------------------------------------------------------------------------
function [xyzPoints, indexPairs] = helperReconstructFromStereo(I1, I2, ...
    features1, features2, points1, points2, stereoParams, currPose, maxDisparity)
%helperReconstructFromStereo reconstruct scene from stereo image using the disparity map

indexPairs     = helperFindValidFeaturePairs(features1, features2, points1, points2, maxDisparity);
disparityMap   = disparitySGM(rgb2gray(I1), rgb2gray(I2), 'DisparityRange', [0, maxDisparity], ...
    'UniquenessThreshold', 2);
xyzPointsAll   = reconstructScene(disparityMap, stereoParams);

% Find the corresponding world point of the matched feature points 
locations      = floor(points1.Location(indexPairs(:, 1), [2 1]));
xyzPoints      = [];
isPointFound   = false(size(points1));

for i = 1:size(locations, 1)
    point3d = squeeze(xyzPointsAll(locations(i,1), locations(i, 2), :))';
    isPointValid   = all(~isnan(point3d)) && all(isfinite(point3d)) &&  point3d(3) > 0;
    isDepthInRange = point3d(3) < 100*abs(stereoParams.TranslationOfCamera2(1));
    if isPointValid && isDepthInRange
        xyzPoints       = [xyzPoints; point3d]; %#ok<*AGROW> 
        isPointFound(i) = true;
    end
end
indexPairs = indexPairs(isPointFound, :);
xyzPoints  = xyzPoints * currPose.Rotation + currPose.Translation;
end

%--------------------------------------------------------------------------
function indexPairs = helperFindValidFeaturePairs(features1, features2, points1, points2, maxDisparity)
%helperFindValidFeaturePairs match features between a pair of stereo images

indexPairs  = matchFeatures(features1, features2,...
    'Unique', true, 'MaxRatio', 1, 'MatchThreshold', 40);

matchedPoints1 = points1.Location(indexPairs(:,1), :);
matchedPoints2 = points2.Location(indexPairs(:,2), :);
scales1        = points1.Scale(indexPairs(:,1), :);
scales2        = points2.Scale(indexPairs(:,2), :);

dist2EpipolarLine = abs(matchedPoints2(:, 2) - matchedPoints1(:, 2));
shiftDist = matchedPoints1(:, 1) - matchedPoints2(:, 1);

isCloseToEpipolarline = dist2EpipolarLine < 2*scales2;
isDisparityValid      = shiftDist > 0 & shiftDist < maxDisparity;
isScaleIdentical      = scales1 == scales2;
indexPairs = indexPairs(isCloseToEpipolarline & isDisparityValid & isScaleIdentical, :);
end

%--------------------------------------------------------------------------
function isKeyFrame = helperIsKeyFrame(mapPoints, ...
    refKeyFrameId, lastKeyFrameIndex, currFrameIndex, mapPointsIndices)
%helperIsKeyFrame check if a frame is a key frame

numPointsRefKeyFrame = numel(findWorldPointsInView(mapPoints, refKeyFrameId));

% More than 15 frames have passed from last key frame insertion
tooManyNonKeyFrames = currFrameIndex >= lastKeyFrameIndex + 15;

% Track less than 100 map points
tooFewMapPoints     = numel(mapPointsIndices) < 100;

% Tracked map points are fewer than 90% of points tracked by
% the reference key frame
tooFewTrackedPoints = numel(mapPointsIndices) < 0.9 * numPointsRefKeyFrame;

isKeyFrame = ((tooManyNonKeyFrames && tooFewTrackedPoints) || tooFewMapPoints);
end

%--------------------------------------------------------------------------
function [mapPointSet, directionAndDepth, mapPointsIdx] = ...
    helperCullRecentMapPoints(mapPointSet, directionAndDepth, mapPointsIdx, newPointIdx, stereoMapPointsIndices)
%helperCullRecentMapPoints cull recently added map points
outlierIdx = setdiff([newPointIdx; stereoMapPointsIndices], mapPointsIdx);

if ~isempty(outlierIdx)
    mapPointSet   = removeWorldPoints(mapPointSet, outlierIdx);
    directionAndDepth = remove(directionAndDepth, outlierIdx);
    mapPointsIdx  = mapPointsIdx - arrayfun(@(x) nnz(x>outlierIdx), mapPointsIdx);
end

end

%--------------------------------------------------------------------------
function [mapPointSet, directionAndDepth] = helperUpdateGlobalMap(...
    mapPointSet, directionAndDepth, vSetKeyFrames, vSetKeyFramesOptim)
%helperUpdateGlobalMap update map points after pose graph optimization
posesOld     = vSetKeyFrames.Views.AbsolutePose;
posesNew     = vSetKeyFramesOptim.Views.AbsolutePose;
positionsOld = mapPointSet.WorldPoints;
positionsNew = positionsOld;
indices = 1:mapPointSet.Count;

% Update world location of each map point based on the new absolute pose of 
% the corresponding major view
for i = 1: mapPointSet.Count
    majorViewIds = directionAndDepth.MajorViewId(i);
    tform = posesOld(majorViewIds).T \ posesNew(majorViewIds).T ;
    positionsNew(i, :) = positionsOld(i, :) * tform(1:3,1:3) + tform(4, 1:3);
end
mapPointSet = updateWorldPoints(mapPointSet, indices, positionsNew);
end