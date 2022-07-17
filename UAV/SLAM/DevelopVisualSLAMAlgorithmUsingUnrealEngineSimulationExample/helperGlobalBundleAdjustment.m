function [vSetKeyFrames, mapPointSet, directionAndDepth] = helperGlobalBundleAdjustment(...
    vSetKeyFrames, mapPointSet, directionAndDepth, intrinsics, varargin)

%   This is an example helper function that is subject to change or removal 
%   in future releases.

%   Copyright 2019-2021 The MathWorks, Inc.

% Run full bundle adjustment on the first two key frames
tracks       = findTracks(vSetKeyFrames);
cameraPoses  = poses(vSetKeyFrames);

[pointIdx, validIdx] = findWorldPointsInTracks(mapPointSet, tracks);
xyzWorldPoints = mapPointSet.WorldPoints(pointIdx, :);
tracks = tracks(validIdx);

[refinedPoints, refinedAbsPoses] = bundleAdjustment(xyzWorldPoints, tracks, ...
    cameraPoses, intrinsics, 'FixedViewIDs', 1, ...
    'PointsUndistorted', true, 'AbsoluteTolerance', 1e-7,...
    'RelativeTolerance', 1e-16, 'MaxIteration', 50, ...
    'Solver', 'preconditioned-conjugate-gradient');

% Scale the map and the camera pose using the median depth of map points
medianDepth   = median(vecnorm(refinedPoints.'));
refinedPoints = refinedPoints / medianDepth;

if nargin > 4
    relPose = varargin{1};
    refinedAbsPoses.AbsolutePose(end).Translation = ...
        refinedAbsPoses.AbsolutePose(end).Translation / medianDepth;
    relPose.Translation = relPose.Translation/medianDepth;
    vSetKeyFrames = updateConnection(vSetKeyFrames, 1, 2, relPose);
end

% Update key frames with the refined poses
vSetKeyFrames = updateView(vSetKeyFrames, refinedAbsPoses);

% Update map points with the refined positions
mapPointSet   = updateWorldPoints(mapPointSet, pointIdx, refinedPoints);

% Update view direction and depth 
directionAndDepth = update(directionAndDepth, mapPointSet, vSetKeyFrames.Views, pointIdx, true);