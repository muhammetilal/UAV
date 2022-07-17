classdef helperLidarObjecDetector < handle
    % helperLidarObjecDetector Helper class to perform object detection and
    % classification on lidar data
    
    %  Copyright 2020-2021 The MathWorks, Inc.
    
    properties
        CurrentPtCloud;
    end
    properties (Access = private)
        % String to store path of trained model file
        Model = [];
        
        % Range in x direction
        XRange;
        
        % Range in y direction
        YRange;
        
        % Range in z direction
        ZRange;
        
        % Segmentation Network
        Net;
        
        ROI;        
    end
    properties (Access = private, Constant)
        % Default color map for semantic segmentation output label
        LidarColorMap = [
            0.0  0.0   0.0  % unknown
            0.01  0.98   0.01  % green color for car
            0.01  0.01   0.98  % blue color for motorcycle
            ];        
        
        % Default reference vector for RANSAC based plane segmentation
        DefaultReferenceVector = [0, 0, 1];
        
        % Default range in X direction
        DefaultXRange = [-30, 30]; % x range to crop display
        
        % Default range in y direction
        DefaultYRange = [-12, 12];% y range to crop display
        
        % Default range in z direction
        DefaultZRange = [-3, 15];
        
        ReferenceVector = [0, 0, 1];
    end
    
    methods
        function this = helperLidarObjecDetector(varargin)
            parser = inputParser;
            validScalarPosNum = @(x)(isequal(size(x), [1, 2]));
            addParameter(parser, 'Model', this.Model);
            addOptional(parser, 'XLimits', this.DefaultXRange, ...
                validScalarPosNum);
            addOptional(parser, 'YLimits', this.DefaultYRange, ...
                validScalarPosNum);
            addOptional(parser, 'ZLimits', this.DefaultZRange, ...
                validScalarPosNum);
            parse(parser, varargin{:});
            
            this.Model = parser.Results.Model;
            this.XRange = parser.Results.XLimits;
            this.YRange = parser.Results.YLimits;
            this.ZRange = parser.Results.ZLimits;
            
            % Load semantic segmentation network
            if ~isempty(this.Model)
                this.Net = this.Model;
            end
            
            this.ROI = [this.XRange, this.YRange, this.ZRange];
        end
        
        function [boundingBox, coloredCloud, SegmentedImage] = ...
                                                detectBbox(this, inputData)            
            % Run Segmentation over the possible objects
            [carIndices, truckIndices, SegmentedImage] = this.segmentation(inputData);            
            
            % Extract ground and non ground point index
            this.CurrentPtCloud = pointCloud(inputData(:, :, 1:3));
            [nonGroundIdx, groundIdx] = this.extractGround;
                        
            % Get bounding box parameters
            boundingBox = this.findBoundingBoxes(nonGroundIdx, carIndices, truckIndices);
            
            % Fill color to point cloud
            coloredCloud = this.fillColor(nonGroundIdx, groundIdx);
        end
    end
        
    methods (Access = private)
        function [carIndices, truckIndices, segmentedImage] = ...
                segmentation(this, inputPointCloud)
            %segmentation Method for running segmentation on organized
            %   point cloud data
            
            pxdsResults = semanticseg(inputPointCloud, this.Net);
            segmentedImage = ...
                labeloverlay(uint8(inputPointCloud(:, :, 4)), ...
                pxdsResults, 'Colormap', this.LidarColorMap, ...
                'Transparency', 0.5);
            
            %Extract indices from labels
            carIndices = pxdsResults == 'car';
            truckIndices = pxdsResults == 'truck';
        end
        
        function coloredPtCloud = fillColor(this, nonGroundIdx, groundIdx)
              
            % Reshape point cloud
            colorMap = 255*repmat(ones(1, 3), this.CurrentPtCloud.Count, 1);
                        
            % Magenta color for ground
            colorMap(groundIdx, 2) = 0;
            
            % Cyan color for ground
            colorMap(nonGroundIdx, 1) = 0;
            
            colorMap = reshape(colorMap, size(this.CurrentPtCloud.Location));
            
            coloredPtCloud = this.CurrentPtCloud;
            coloredPtCloud.Color = uint8(colorMap);     
        end
        
        function boundingBox = findBoundingBoxes(this, nonGroundIdx, carIndices, truckIndices)
            % Extract non ground point cloud
            nonGround = select(this.CurrentPtCloud, nonGroundIdx, 'Output', 'Full');
            
            % Filter point clouds with car and truck separately
            carPointCloud = select(nonGround, carIndices);
            truckPointCloud = select(nonGround, truckIndices);
            
            % Cluster point cloud with car
            [labelsCar, numClustersCar] = pcsegdist(carPointCloud, 0.5);
            
            % Cluster point cloud with truck
            [labelsTruck, numClustersTruck] = pcsegdist(truckPointCloud, 1);
            
            boundingBox = zeros(0, 10);
            
            for clusterIndex = 1:numClustersCar
                ptsInCluster = labelsCar == clusterIndex;
                
                % Since size is not kept same so the output is unorganized
                % point cloud
                pc = select(carPointCloud, ptsInCluster);
                location = pc.Location;
                xl = (max(location(:, 1)) - min(location(:, 1)));
                if size(location, 1)*size(location, 2) > 20 && any(any(pc.Location)) && xl > 1                    
                    indices = regionGrowing(nonGround, location, 30);
                    model = pcfitcuboid(select(nonGround, indices));
                    boundingBox(end+1, :) = [model.Parameters, 1]; %#ok<AGROW>
                end
            end
            
            for clusterIndex = 1:numClustersTruck
                ptsInCluster = labelsTruck == clusterIndex;
                
                % Since size is not kept same so the output is unorganized
                % point cloud
                pc = select(truckPointCloud, ptsInCluster);
                location = pc.Location;
                xl = (max(location(:, 1)) - min(location(:, 1)));
                yl = (max(location(:, 2)) - min(location(:, 2)));
                if size(location, 1)*size(location, 2) > 20 && any(any(pc.Location)) && xl > 1 && yl > 1                                        
                    indices = regionGrowing(nonGround, location, 50);
                    model = pcfitcuboid(select(nonGround, indices));
                    boundingBox(end+1, :) = [model.Parameters, 2]; %#ok<AGROW>
                end
            end
        end
        
        function [nonGroundIdx, groundIdx] = extractGround(this)
            warning('off','pointcloud:notEnoughPts');
            warning('off','vision:pointcloud:notEnoughInliers');
            % Remove ego vehicle points            
            sensorOrigin = [0 0 0];
            radius       = 2;       % in meters
            this.CurrentPtCloud = select(this.CurrentPtCloud, ...
                setdiff(1:this.CurrentPtCloud.Count,findNeighborsInRadius(this.CurrentPtCloud, ...
                sensorOrigin, radius)), 'OutputSize', 'full');
            
            % Crop the point cloud
            idx = findPointsInROI(this.CurrentPtCloud, this.ROI);
            pc = select(this.CurrentPtCloud, idx, 'OutputSize', 'full');
            
            % Get the ground plane the indices using piecewise plane fitting
            groundIdx = piecewisePlaneFitting(this, pc);
            
            nonGroundIdx = false(this.CurrentPtCloud.Count, 1);
            nonGroundIdx(idx) = true;
            nonGroundIdx(groundIdx) = false;
            nonGroundIdx = find(nonGroundIdx);
        end        
        
        function idx = piecewisePlaneFitting(this, ptCloudIn)            
            groundPtsIdx = ...
                segmentGroundFromLidarData(ptCloudIn, ...
                'ElevationAngleDelta', 5, 'InitialElevationAngle', 15);
            groundPC = select(ptCloudIn,groundPtsIdx, 'OutputSize', 'full');
            
            % Divide x-axis in 3 regions
            segmentLength = (this.ROI(2) - this.ROI(1))/3;
            
            x1 = [this.ROI(1), this.ROI(1) + segmentLength];
            x2 = [x1(2), x1(2) + segmentLength];
            x3 = [x2(2), x2(2) + segmentLength];
            
            roi1 = [x1, this.ROI(3: end)];
            roi2 = [x2, this.ROI(3: end)];
            roi3 = [x3, this.ROI(3: end)];
            
            idxBack = findPointsInROI(groundPC, roi1);
            idxCenter = findPointsInROI(groundPC, roi2);
            idxForward = findPointsInROI(groundPC, roi3);
            
            % Break the point clouds in front and back
            ptBack = select(groundPC, idxBack, 'OutputSize', 'full');
            
            ptForward = select(groundPC, idxForward, 'OutputSize', 'full');
            
            [~, inliersForward] = this.planeFit(ptForward);
            [~, inliersBack] = this.planeFit(ptBack);
            idx = [inliersForward; idxCenter; inliersBack];
        end
        
        function [plane, inlinersIdx] = planeFit(this, ptCloudIn)            
            [~, inlinersIdx, ~] = pcfitplane(ptCloudIn, 1, this.ReferenceVector);
            plane = select(ptCloudIn, inlinersIdx, 'OutputSize', 'full');
        end
    end
end

function indices = regionGrowing(ptCloud, seedPoints, numNeighbours)
indices = zeros(0, 1);
for i = 1:size(seedPoints, 1)
    seedPoint = seedPoints(i, :);
    indices = [indices;findNearestNeighbors(ptCloud, seedPoint, numNeighbours)]; %#ok<AGROW> 
    %indices(end+1) = findNearestNeighbors(ptCloud, seedPoint, numNeighbours);
end
% Remove overlapping indices
indices = unique(indices);
end