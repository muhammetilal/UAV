function [ptCloudNonGround,ptCloudGround] = helperExtractGround(ptCloudIn,roi)
% Crop the point cloud

idx = findPointsInROI(ptCloudIn,roi);
pc = select(ptCloudIn,idx,'OutputSize','full');

% Get the ground plane the indices using piecewise plane fitting
[ptCloudGround,idx] = piecewisePlaneFitting(pc,roi);

nonGroundIdx = true(size(pc.Location,[1,2]));
nonGroundIdx(idx) = false;
ptCloudNonGround = select(pc,nonGroundIdx,'OutputSize','full');
end


function [groundPlane,idx] = piecewisePlaneFitting(ptCloudIn,roi)
groundPtsIdx = ...
    segmentGroundFromLidarData(ptCloudIn, ...
    'ElevationAngleDelta',5,'InitialElevationAngle',15);
groundPC = select(ptCloudIn,groundPtsIdx,'OutputSize','full');

% Divide x-axis in 3 regions
segmentLength = (roi(2) - roi(1))/3;

x1 = [roi(1),roi(1) + segmentLength];
x2 = [x1(2),x1(2) + segmentLength];
x3 = [x2(2),x2(2) + segmentLength];

roi1 = [x1,roi(3:end)];
roi2 = [x2,roi(3:end)];
roi3 = [x3,roi(3:end)];

idxBack = findPointsInROI(groundPC,roi1);
idxCenter = findPointsInROI(groundPC,roi2);
idxForward = findPointsInROI(groundPC,roi3);

% Break the point clouds in front and back
ptBack = select(groundPC,idxBack,'OutputSize','full');

ptForward = select(groundPC,idxForward,'OutputSize','full');

[~,inliersForward] = planeFit(ptForward);
[~,inliersBack] = planeFit(ptBack);
idx = [inliersForward; idxCenter; inliersBack];
groundPlane = select(ptCloudIn, idx,'OutputSize','full');
end

function [plane,inlinersIdx] = planeFit(ptCloudIn)
[~,inlinersIdx, ~] = pcfitplane(ptCloudIn,1,[0, 0, 1]);
plane = select(ptCloudIn,inlinersIdx,'OutputSize','full');
end