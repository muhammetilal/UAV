function ptCloud = helperProcessPointCloud(ptCloud, params)
%helperProcessPointCloud Process point cloud
%   helperProcessPointCloud processes a lidar point cloud by only keeping
%   points within some radius of the sensor and removing the ground plane.
%
%   See also pointCloud, findNeighborsInRadius, segmentGroundFromLidarData.

% Copyright 2020 The MathWorks, Inc.

arguments
    ptCloud(1,1) pointCloud
    params.MinRadius(1,1)       double  {mustBeNumeric} = 2;
    params.MaxRadius(1,1)       double  {mustBeNumeric} = 25;
    params.RemoveGround(1,1)    logical {mustBeNumericOrLogical} = true; 
end
% Discard points too close to the sensor
sensorOrigin = [0 0 0];
minRadius    = params.MinRadius;       % in meters
ptCloud = select(ptCloud, ...
    setdiff(1:ptCloud.Count,findNeighborsInRadius(ptCloud, sensorOrigin, minRadius)), ...
    'OutputSize', 'full');

% Discard points too far away
sensorOrigin = [0 0 0];
maxRadius    = params.MaxRadius;      % in meters
ptCloud = select(ptCloud, ...
    findNeighborsInRadius(ptCloud, sensorOrigin, maxRadius), ...
    'OutputSize', 'full');

% Segment and remove ground
removeGroundFlag = params.RemoveGround;
if removeGroundFlag
    ptCloud = select(ptCloud, ...
        ~segmentGroundFromLidarData(ptCloud), 'OutputSize', 'full');
end
end