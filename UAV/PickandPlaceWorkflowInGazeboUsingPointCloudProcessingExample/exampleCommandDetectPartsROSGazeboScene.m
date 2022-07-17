function exampleCommandDetectPartsROSGazeboScene(coordinator)

% Detect parts and identify their poses

% Copyright 2020 The MathWorks, Inc.

% This function detects parts using a depth camera. Each
% part is a struct with 2 elements: centerPoint and type.
% Part of type '2' is a can. Part of type '1' is a bottle.
% We assume the CAD files of both bottle and can are available. 

    % Empty cell array of parts to detect new parts
    coordinator.Parts = {};
    
    % Load can and bottle STL files
    bottle = stlread('exampleHelperBottlePoints.stl');
    can =  stlread('exampleHelperCanPoints.stl');
    
    % Generate point clouds from STL 
    pcBottle = pointCloud(bottle.Points);
    pcCan = pointCloud(can.Points);

    % Register captured segmented point clouds to known point clouds of
    % parts to pick
    for label=1:numel(coordinator.PointCloudSegments)         
        object = coordinator.PointCloudSegments{label};       
        [tformsBottle(label),objectTransformed,rmseBottle(label)] = pcregistericp(object,pcBottle, 'Metric','pointToPoint');
        [tformsCan(label),objectTransformed,rmseCan(label)] = pcregistericp(object,pcCan,'Metric','pointToPoint');       
    end
    
    % Identify bottle
    [~, bottleLabel] = min(rmseBottle(rmseBottle>0));
    part.index = bottleLabel;
    part.type = 1;
    part.pointcloud = coordinator.PointCloudSegments{bottleLabel};
    part.mesh = coordinator.World{bottleLabel};
    disp("Bottle detected...");
    % Compute bottle centroid 
    tformBottle = invert(tformsBottle(bottleLabel));
    pcBottleCadWorld = pctransform(pcBottle,tformBottle);
    centroidBottleCad = mean(pcBottleCadWorld.Location);
    part.centerPoint = centroidBottleCad ;
    % Add to list of Parts
    coordinator.Parts{1} = part;
    
    % Identify can
    [~, canLabel] = min(rmseCan(rmseCan>0));
    part.index = canLabel;
    part.type = 2;
    part.pointcloud = coordinator.PointCloudSegments{canLabel};
    part.mesh = coordinator.World{canLabel};
    disp("Can detected..."); 
    % Compute can centroid
    tformCan = invert(tformsCan(canLabel));
    pcCanCadWorld = pctransform(pcCan,tformCan);
    centroidCanCad = mean(pcCanCadWorld.Location);
    part.centerPoint = centroidCanCad;
    % Add to list of Parts
    coordinator.Parts{2} = part;
    
    coordinator.NextPart = 0;
    if ~isempty(coordinator.Parts) 
        coordinator.DetectedParts = coordinator.Parts;
        return;
    end
    coordinator.NumDetectionRuns = coordinator.NumDetectionRuns +1;
end