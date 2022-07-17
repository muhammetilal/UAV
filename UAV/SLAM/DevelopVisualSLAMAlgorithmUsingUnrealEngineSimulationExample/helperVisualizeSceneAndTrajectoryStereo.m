classdef helperVisualizeSceneAndTrajectoryStereo < handle
%helperVisualizeSceneAndTrajectoryStereo show map points and camera trajectory
%
%   This is an example helper class that is subject to change or removal 
%   in future releases.

%   Copyright 2020 The MathWorks, Inc.

    properties
        XLim = [-10 30]
        
        YLim = [-10 10]
        
        ZLim = [-20 80]
        
        Axes
    end
    
    properties (Access = private)
        MapPointsPlot
 
        EstimatedTrajectory
        
        OptimizedTrajectory

        CameraPlot
    end
    
    methods (Access = public)
        function obj = helperVisualizeSceneAndTrajectoryStereo(vSetKeyFrames, mapPoints)
        
            [xyzPoints, currPose, trajectory]  = retrievePlottedData(obj, vSetKeyFrames, mapPoints);
             
            obj.MapPointsPlot = pcplayer(obj.XLim, obj.YLim, obj.ZLim, ...
                'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
            
            obj.Axes  = obj.MapPointsPlot.Axes;
            
            % Set figure location
            obj.Axes.Parent.Position = [100 100 800 350];
            
            color = xyzPoints(:, 2);
            color = -min(2, max(-10, color));
            obj.MapPointsPlot.view(xyzPoints, color);
            obj.Axes.Children.DisplayName = 'Map points';
            
            hold(obj.Axes, 'on');
            
            % Set figure position on the screen
            movegui(obj.Axes.Parent, [1200 200]);
            
            % Plot camera trajectory
            obj.EstimatedTrajectory = plot3(obj.Axes, trajectory(:,1), trajectory(:,2), ...
                trajectory(:,3), 'r', 'LineWidth', 2 , 'DisplayName', 'Estimated trajectory');
            
            % Plot the current cameras
            obj.CameraPlot = plotCamera(currPose, 'Parent', obj.Axes, 'Size', 1);
            view(obj.Axes, [0 -1 0]);
            camroll(obj.Axes, 90);
        end
        
        function updatePlot(obj, vSetKeyFrames, mapPoints)
            
            [xyzPoints, currPose, trajectory]  = retrievePlottedData(obj, vSetKeyFrames, mapPoints);
            
            % Update the point cloud
            color = xyzPoints(:, 2);
            color = -min(2, max(-10, color));
            obj.MapPointsPlot.view(xyzPoints, color);
            
            % Update the camera trajectory
            set(obj.EstimatedTrajectory, 'XData', trajectory(:,1), 'YData', ...
                trajectory(:,2), 'ZData', trajectory(:,3));
            
            % Update the current camera pose since the first camera is fixed
            obj.CameraPlot.AbsolutePose = currPose.AbsolutePose;
            obj.CameraPlot.Label        = num2str(currPose.ViewId);
            
            drawnow limitrate
        end
        
        function plotOptimizedTrajectory(obj, poses)
            
            % Delete the camera plot
            delete(obj.CameraPlot);
            
            % Plot the optimized trajectory
            trans = vertcat(poses.AbsolutePose.Translation);
            obj.OptimizedTrajectory = plot3(obj.Axes, trans(:, 1), trans(:, 2), trans(:, 3), 'm', ...
                'LineWidth', 2, 'DisplayName', 'Optimized trajectory');
        end
        
        function optimizedLocations = plotActualTrajectory(obj, gTruth)
            
            actualCameraLoc = vertcat(gTruth.Translation);

            % Transform to the global coordinate system
            optimizedLocations = transformCoodinates(obj, gTruth(1));
            
            % Plot the ground truth
            plot3(obj.Axes, actualCameraLoc(:,1), actualCameraLoc(:,2), actualCameraLoc(:,3), ...
                'g','LineWidth',2, 'DisplayName', 'Actual trajectory');
            view(obj.Axes, [0 0 1]);
            drawnow limitrate
        end
        
        function showLegend(obj)
            % Add a legend to the axes
            hLegend = legend(obj.Axes, 'Location',  'northwest', ...
                'TextColor', [1 1 1], 'FontWeight', 'bold');
        end
    end
    
    methods (Access = private)
        function scaledLocations = transformCoodinates(obj, initialPose)
            % Update the map points and camera trajectory based on the
            % initial pose of the sensor
            
            imageToCamera = rigid3d([0 -1 0 0; 0 0 -1 0; 1 0 0 0; 0 0 0 1]);
            tform = rigid3d(imageToCamera.T * initialPose.T);
            
            preLim = [obj.Axes.XLim.', obj.Axes.YLim.', obj.Axes.ZLim.'];
            currLim = transformPointsForward(tform, preLim);
            obj.Axes.XLim = [min(currLim(:,1)), max(currLim(:,1))];
            obj.Axes.YLim = [min(currLim(:,2)), max(currLim(:,2))];
            obj.Axes.ZLim = [min(currLim(:,3)), max(currLim(:,3))];
            
            % Map points
            [mapX, mapY, mapZ] = transformPointsForward(tform, ...
                obj.Axes.Children(end).XData, ...
                obj.Axes.Children(end).YData, ...
                obj.Axes.Children(end).ZData);
            obj.Axes.Children(end).XData = mapX;
            obj.Axes.Children(end).YData = mapY;
            obj.Axes.Children(end).ZData = mapZ;
            
            % Estimated and optimized Camera trajectory
            [estimatedTrajX, estimatedTrajY, estimatedTrajZ] = ...
                transformPointsForward(tform, ...
                obj.EstimatedTrajectory.XData, ...
                obj.EstimatedTrajectory.YData, ...
                obj.EstimatedTrajectory.ZData);
            obj.EstimatedTrajectory.XData = estimatedTrajX;
            obj.EstimatedTrajectory.YData = estimatedTrajY;
            obj.EstimatedTrajectory.ZData = estimatedTrajZ;
            
            [optimizedTrajX, optimizedTrajY, optimizedTrajZ] = ...
                transformPointsForward(tform, ...
                obj.OptimizedTrajectory.XData, ...
                obj.OptimizedTrajectory.YData, ...
                obj.OptimizedTrajectory.ZData);
            obj.OptimizedTrajectory.XData = optimizedTrajX;
            obj.OptimizedTrajectory.YData = optimizedTrajY;
            obj.OptimizedTrajectory.ZData = optimizedTrajZ;
            
            scaledLocations = [optimizedTrajX.', optimizedTrajY.', optimizedTrajZ.'];
        end
        
        function [xyzPoints, currPose, trajectory]  = retrievePlottedData(obj, vSetKeyFrames, mapPoints)
            camPoses    = poses(vSetKeyFrames);
            currPose    = camPoses(end,:); % Contains both ViewId and Pose
            trajectory  = vertcat(camPoses.AbsolutePose.Translation);
            xyzPoints   = mapPoints.WorldPoints;%(mapPoints.UserData.Validity,:);
            
            % Only plot the points within the limit
            inPlotRange = xyzPoints(:, 1) > obj.XLim(1) & ...
                xyzPoints(:, 1) < obj.XLim(2) & xyzPoints(:, 2) > obj.YLim(1) & ...
                xyzPoints(:, 2) < obj.YLim(2) & xyzPoints(:, 3) > obj.ZLim(1) & ...
                xyzPoints(:, 3) < obj.ZLim(2);
            xyzPoints   = xyzPoints(inPlotRange, :);
        end
    end
end

