classdef helperLidarObjectDetectionDisplay < handle
    %helperLidarObjectDetectionDisplay helper class to display segmented
    %   point cloud data with oriented bounding box visualization.
    
    % Copyright 2019-2021 The Mathworks Inc.
    
    properties
        PositionIndex = [1 3 6];
        VelocityIndex = [2 4 7];
        DimensionIndex = [9 10 11];
        YawIndex = 8;
    end
    
    properties (Access = private)
        % Image handle for visualizing segmented 2D images of organized
        % point cloud
        SegmentedImageH;
        
        % Axes handle for point cloud visualization
        PcPlotAxes;
        
        % Axes handle for point cloud with oriented bounding box
        % visualization
        PcPlotBboxAxes;
        
        % Scatter plot handle for visualizing segmented colored point cloud
        % from network
        ScatterPlotH;
        
        % Scatter plot handle for visualizing point cloud with oriented
        % bounding box
        ScatterPlotWithBboxH;
        
        % Image handle for visualizing ground truth labels
        Lidar2DImageH;
        
        % Rgb image axes handle
        ImageAxesH;
        
        % Property for holding all the holding frames from figure handle
        % frames used for writing output video
        PFrames;
        
        % Main gui figure handle
        FigureH;
        
        % Range in x direction
        XRange;
        
        % Range in y direction
        YRange;
        
        % Range in z direction
        ZRange;
        
        % Set it if want to write a video output for the results
        WriteVideo
    end
    
    properties (Constant, Access = protected)
        CuboidFaces = [1 5 8 4; ... %far
            2 3 7 6; ... %near
            5 6 7 8; ... %top
            2 1 4 3; ... %bot
            3 4 8 7; ... %left
            5 6 2 1; ... %right
            ];
        
        % Calibration matrix used for projecting 3D point to image
        Calibration=[];
        
        % Default range in X direction
        DefaultXRange = [-30,30]; % x range to crop display
        
        % Default range in y direction
        DefaultYRange =[-12,12];% y range to crop display
        
        % Default range in z direction
        DefaultZRange =[-3,8];
        
        % Default value to write video
        DefaultWriteVideo = false;
    end
    
    methods
        %--------------------------------------------------------------------------
        function obj = helperLidarObjectDetectionDisplay(varargin)
            %% Parse name value parser argument
            parser = inputParser;
            parser.CaseSensitive = false;
            
            % Parameter Validator
            validScalarPosNum = @(x)  (isequal(size(x),[1,2]));
            validCalibration = @(x)  (isequal(size(x),[3,4]));
            validVideo = @(x) (islogical(x));
            % Optional argument parsers
            addOptional(parser,'XLimits',obj.DefaultXRange,validScalarPosNum);
            addOptional(parser,'YLimits',obj.DefaultYRange,validScalarPosNum);
            addOptional(parser,'ZLimits',obj.DefaultZRange,validScalarPosNum);
            addOptional(parser,'Calibration',obj.Calibration,validCalibration);
            addOptional(parser,'Video',obj.DefaultWriteVideo,validVideo);
            % Parse input arguments
            parse(parser,varargin{:});
            obj.XRange=parser.Results.XLimits;
            obj.YRange=parser.Results.YLimits;
            obj.ZRange=parser.Results.ZLimits;
            obj.WriteVideo =  parser.Results.Video;
            
            
            %% Initialize main figure
            obj.FigureH = figure('Visible','off','Position',[0, 0, 1200, 640],...
                'Name','Object Detection','color',[0 0 0],'InvertHardcopy','off');
            
            %% Initialize panel for  point cloud visualization
            pointCloudPanel = uipanel('Parent',obj.FigureH,'Position',[0.01,0,0.50,0.58],...
                'BackgroundColor',[0 0 0],'Title','Oriented Bounding Box Detection',...
                'ForegroundColor',[1,1,1],'FontSize',15);
            
            obj.PcPlotAxes = axes('Parent',pointCloudPanel,'Color',[0 0 0],...
                'Position',[0,0,1,1],'NextPlot','replacechildren',...
                'XLim',obj.XRange,'YLim',obj.YRange,'ZLim',obj.ZRange);
            
            axis(obj.PcPlotAxes,'equal');
            obj.PcPlotAxes.XLimMode = 'manual';
            obj.PcPlotAxes.YLimMode = 'manual';
            obj.PcPlotAxes.ZLimMode = 'manual';
            obj.ScatterPlotH = scatter3(obj.PcPlotAxes,nan,nan,nan,  7,  '.');
            view(obj.PcPlotAxes,3);
            campos([-169.3414 -242.6887  131.6010]);
            
            %% Bounding box point cloud visualizationWriteVideo
            pointCloudPanel1 = uipanel('Parent',obj.FigureH,'Position',[0.5,0,0.50,0.58],...
                'BackgroundColor',[0 0 0],'Title','Top View'...
                ,'ForegroundColor',[1,1,1],'FontSize',15);
            
            obj.PcPlotBboxAxes = axes('Parent',pointCloudPanel1,'Color',[0 0 0],...
                'NextPlot','replacechildren','Position' , [0,0,1,1],...
                'XLim',obj.XRange,'YLim',obj.YRange,'ZLim',obj.ZRange);
            
            obj.PcPlotBboxAxes.Title.Color = [1 1 1];
            axis(obj.PcPlotBboxAxes,'equal');
            obj.PcPlotBboxAxes.XLimMode = 'manual';
            obj.PcPlotBboxAxes.YLimMode = 'manual';
            obj.PcPlotBboxAxes.ZLimMode = 'manual';
            obj.ScatterPlotWithBboxH = scatter3(obj.PcPlotBboxAxes,nan,nan,nan,  7,  '.');
            view(obj.PcPlotBboxAxes,3);
            campos([0 0 323.5756]);
            
            %% Segmented Point CLoud Image visualization
            hFrontView1 = uipanel(obj.FigureH, 'Position', [0.05 0.58 0.90 0.2],...
                'Title','Segmented Image','FontSize',15);
            ImageAxes1 = axes('Parent',hFrontView1);
            obj.SegmentedImageH = imshow([],'Parent',ImageAxes1,'DisplayRange',[0,70]);
            ImageAxes1.NextPlot = 'add';
            ImageAxes1.Position = [0,0,1,1];
            hFrontView1.BackgroundColor = [0 0 0];
            hFrontView1.ForegroundColor = [1 1 1];
            axis(ImageAxes1,'tight');
            
            %% Range Lidar Image visualization
            hFrontView2 = uipanel(obj.FigureH, 'Position', [0.05 0.78 0.90 0.2]...
                ,'Title','Lidar Range Image','FontSize',15);
            ImageAxes2 = axes('Parent',hFrontView2);
            obj.Lidar2DImageH = imshow([],'Parent',ImageAxes2,'DisplayRange',[0,40]);
            ImageAxes2.NextPlot = 'add';
            ImageAxes2.Position = [0,0,1,1];
            hFrontView2.BackgroundColor = [0 0 0];
            hFrontView2.ForegroundColor = [1 1 1];
            colormap(ImageAxes2,'jet');
            axis(ImageAxes2,'tight');
        end
        
        %--------------------------------------------------------------------------
        function updateSegmentedImage(obj, segmentedImageLabel, lidarImage)
            %updateSegmentedImage Method to update display with segmented
            %   image label and lidar range or intensity image.
            %
            %   Inputs:
            %     segmentedImageLabel   Segmented output lidar image from the
            %                           segmentation network
            %
            %     rawInput              Range or Intensity image from lidar
            %                           data
            obj.Lidar2DImageH.CData = lidarImage;
            obj.SegmentedImageH.CData = segmentedImageLabel;
        end
        
        %--------------------------------------------------------------------------
        function  updatePointCloud(obj, pc)
            %updatePC Method to update display with point cloud.
            %   Inputs:
            %     pc                 pointCloud type object
            location = pc.Location;
            s = size(location);
            
            % Check for unorganized point cloud
            if(size(s, 2) < 3)
                color = pc.Color;
                x = location(:, 1);
                y = location(:, 2);
                z = location(:, 3);
                if ~isempty(pc.Color)
                    color = pc.Color;
                elseif ~isempty(pc.Intensity)
                    color = pc.Intensity;
                else
                    color = z;
                end
            else
                color = reshape(pc.Color, [], 3);
                x = reshape(location(:, :, 1), [], 1);
                y = reshape(location(:, :, 2), [], 1);
                z = reshape(location(:, :, 3), [], 1);
                if ~isempty(pc.Color)
                    color = reshape(pc.Color, [], 3);
                elseif ~isempty(pc.Intensity)
                    color = reshape(pc.Intensity, [], 3);
                else
                    color = z;
                end
            end
            
            set(obj.ScatterPlotH, 'XData', x, 'YData', y, 'ZData', z, 'CData', color);
            
            set(obj.ScatterPlotWithBboxH, 'XData', x, 'YData', y, 'ZData', z, 'CData', color);
            
            % get video frames only when video record is on
            if obj.WriteVideo
                obj.PFrames{end+1} = getframe(obj.FigureH);
            end
        end
        
        %--------------------------------------------------------------------------
        function updateTracks(obj, confirmedTracks)
            %updateTracks Method to draw tracked bounding boxes and its labels in the
            %   point cloud.
            %   Inputs:
            %     confirmedTracks     An M-by-3 array representing
            %                         positions([x,y,z]) where labels are
            %                         drawn.
            %
            %     centroid            A string array representing track id's
            %                         to be updated
            %
            %     groundExtent        Z value of the ground to extend the
            %                         bounding box till the ground plane
            
            [pos,~, ~, dims, orients, ~, class] = ...
                parseTracks(confirmedTracks, obj.PositionIndex, obj.VelocityIndex, obj.DimensionIndex, obj.YawIndex);
            yaw = zeros(size(pos, 1), 3);
            yaw(:, 3) = orients';
            bboxes = [pos, dims, yaw, class];
            obj.updateBoundingBox(bboxes);                        
        end
        
        %--------------------------------------------------------------------------
        function myCleanupFun(FigureH)
            close(FigureH);
            clc;
        end
        
        %--------------------------------------------------------------------------
        function updateBoundingBox(obj, params, varargin)
            %drawBBox Method to draw 3D bounding box in point cloud display.
            %
            %   Inputs:
            %
            %     corners           Cell array of dimension 1-by-N, where N
            %                       is the number of detected bounding boxes.
            %                       Each array is of dimension 3-by-8 having
            %                       eight corner points.
            %
            %     detections        Cell array of objectdetections object.
            %
            
            switch nargin
                case 2
                    label = repmat("Car", size(params, 1), 1);
                    idxTruck = params(:, end) == 2;
                    label(idxTruck) = "Truck";
                case 3
                    label = varargin{1};
            end
            delete(findobj(obj.PcPlotBboxAxes.Children, 'Tag', 'boundingbox'));
            delete(findobj(obj.PcPlotAxes.Children, 'Tag', 'boundingbox1'));
            
            colorMap = ones(size(params, 1), 3);
            %label = repmat("Car", size(params, 1), 1);
            if ~isempty(params)
                cuboid = params(:, 1:9);
                colorMap(idxTruck, 3) =  0;
                showShape('Cuboid', cuboid, 'Color', colorMap, ...
                    'Parent', obj.PcPlotBboxAxes, 'Opacity',0.15, 'Label', label);
                showShape('Cuboid', cuboid, 'Color', colorMap, ...
                    'Parent', obj.PcPlotAxes, 'Opacity',0.15, 'Label', label);
            end
        end
        
        function initializeDisplay(obj)
            %initializeDisplay To initialize the display
            %turn on the visibility of display
            set(obj.FigureH, 'Visible', 'on');
        end
    end
    
    methods (Access =public, Hidden)
        
        function writeMovie(obj,fileName)
            %writeMovie Method to write displayed results to video.
            %   Inputs:
            %     fileName       optional argument to specify the
            %                    name of the video file. Default name
            %                    is Results.
            
            if nargin <= 1
                fileName = 'Results';
            end
            
            vidFile = VideoWriter(fileName);
            vidFile.FrameRate =5;
            open(vidFile);
            for k = 1:length(obj.PFrames)
                writeVideo(vidFile,obj.PFrames{k});
            end
            close(vidFile);
        end        
    end
end

function [pos, vel, posCov, dim, orientations, labels, class] = ...
    parseTracks(tracks,posIndex,velIndex,dimIndex,yawIndex,modelProbs)
% Parse tracks for plotting using trackPlotter.
numTracks = numel(tracks);
if numTracks > 0
    stateSize = numel(tracks(1).State);
    allTrackStates = cat(2, tracks.State);
    objAttributes = [tracks.ObjectAttributes];
    class = cat(2, objAttributes.ClassID)';
    posSelector = zeros(3,stateSize);
    posSelector(1, posIndex(1)) = 1;
    posSelector(2, posIndex(2)) = 1;
    posSelector(3, posIndex(3)) = 1;
    [pos, posCov] = getTrackPositions(tracks, posSelector);
    velSelector = zeros(3,stateSize);
    velSelector(1, velIndex(1)) = 1;
    velSelector(2, velIndex(2)) = 1;
    velSelector(3, velIndex(3)) = 1;
    vel = getTrackVelocities(tracks, velSelector);
    dim = zeros(numTracks, 3);
    length = allTrackStates(dimIndex(1), :);
    width = allTrackStates(dimIndex(2), :);
    height = allTrackStates(dimIndex(3), :);
    yaw = allTrackStates(yawIndex, :);
    
    orientations = zeros(1, numTracks);
    for i = 1:numTracks
        dim(i, 1) = length(i);
        dim(i, 2) = width(i);
        dim(i, 3) = height(i);
        orientations(i) = yaw(i);
    end
    strMotion = string([]);
    strLabels = string([]);
    for i = 1:numTracks
        strLabels(i) = string(sprintf('T%0.0f',tracks(i).TrackID));
        if nargin > 5
            strMotion(i) = string(sprintf('T%0.0f\nct = %0.2f\ncv = %0.2f',tracks(i).TrackID,modelProbs(1,i),modelProbs(2,i)));
        else
            strMotion(i) = strLabels(i);
        end
    end
    labels = strLabels;
else
    pos = zeros(0,3);
    posCov = zeros(3,3,0);
    vel = zeros(0,3);
    orientations = 0;
    labels = {};
end
end
