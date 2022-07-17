classdef exampleHelperCoordinatorPickPlaceROSGazeboScene < handle
% This class is for internal use and may be removed in a future release
%
%exampleHelperCoordinatorPickPlaceROSGazebo Class used to run the Stateflow
%chart and setup ROS communication with Gazebo
%   This class is used to control the pick-and-place workflow execution.
%   The class serves two main purposes:
%      1. It holds information about ROS-MATLAB communication (subscribers,
%      publishers etc.)
%      2. It stores all data about the current pick and place job
%
% Copyright 2020 The MathWorks, Inc.

    properties         
        FlowChart
        Robot
        World = {};
        Parts = {};
        DetectedParts = {};
        RobotEndEffector
        CurrentRobotJConfig
        CurrentRobotTaskConfig
        NextPart = 0;
        HomeRobotTaskConfig 
        PlacingPose
        GraspPose
        NumJoints
        NumDetectionRuns = 0;
        ROSinfo
        ScanPoses = {}; 
        MergedPointCloud
        PointCloudSegments = {};
        PathHandle
        WorldPatchHandles = {}; 
    end
    
    methods
        function obj = exampleHelperCoordinatorPickPlaceROSGazeboScene(robot, initialRobotJConfig, robotEndEffector)
            obj.Robot = robot;
            
            % Initialize ROS utilities
            obj.ROSinfo.jointsSub = rossubscriber('/my_gen3/joint_states');
            obj.ROSinfo.configClient = rossvcclient('/gazebo/set_model_configuration');
            obj.ROSinfo.gazeboJointNames = {'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7'}; % joint names of robot model in GAZEBO
            obj.ROSinfo.controllerStateSub = rossubscriber('/my_gen3/gen3_joint_trajectory_controller/state');
            obj.ROSinfo.rgbImgSub = rossubscriber('/camera/color/image_raw');
            obj.ROSinfo.pointCloudSub = rossubscriber('/camera/depth/points');
            
            % Initialize robot configuration in GAZEBO
            configResp = setCurrentRobotJConfig(obj, initialRobotJConfig);
            
            % Unpause GAZEBO physics
            physicsClient = rossvcclient('gazebo/unpause_physics');
            physicsResp = call(physicsClient,'Timeout',3);
            
            % Update robot properties
            obj.CurrentRobotJConfig = getCurrentRobotJConfig(obj);
            obj.RobotEndEffector = robotEndEffector;
            obj.CurrentRobotTaskConfig = getTransform(obj.Robot, obj.CurrentRobotJConfig, obj.RobotEndEffector);
            obj.NumJoints = numel(obj.CurrentRobotJConfig);
            
            % Specify scanning poses for Build World task
            obj.ScanPoses ={trvec2tform([0.5, 0.25, 0.45])*axang2tform([0 0 1 -pi/2])*axang2tform([0 1 0 pi]),...
            trvec2tform([0.5, -0.25, 0.45])*axang2tform([0 0 1 -pi/2])*axang2tform([0 1 0 pi]),...
            trvec2tform([0.4, 0.3, 0.45])*axang2tform([0 0 1 pi/2])*axang2tform([0 1 0 pi]),...
            trvec2tform([0.4, -0.1, 0.45])*axang2tform([0 0 1 pi/2])*axang2tform([0 1 0 pi]),...
            trvec2tform([0.4, -0.3, 0.45])*axang2tform([0 0 1 pi/2])*axang2tform([0 1 0 pi])};

            % Initialize visualization
            figure; 
            show(obj.Robot, obj.CurrentRobotJConfig,'PreservePlot', false, 'Frames', 'off');
            hold on
            axis([-0.1 1 -1 1 -0.1 1.2]);
            view(gca,[90.97 50.76]);
            set(gca,'CameraViewAngle',4.25,'DataAspectRatio',[1 1 1],...
                'PlotBoxAspectRatio',[1 1.81 1.18],'Projection',...
                'perspective'); 
        end
        
        function JConfig = getCurrentRobotJConfig(obj)
            jMsg = receive(obj.ROSinfo.jointsSub);
            JConfig =  jMsg.Position(2:8)';
        end
        
        function configResp = setCurrentRobotJConfig(obj, JConfig)            
            configReq = rosmessage(obj.ROSinfo.configClient);
            configReq.ModelName = "my_gen3";
            configReq.UrdfParamName = "/my_gen3/robot_description";
            configReq.JointNames = obj.ROSinfo.gazeboJointNames;
            configReq.JointPositions = JConfig; 
            configResp = call(obj.ROSinfo.configClient, configReq, 'Timeout', 3);
        end
        
        function isMoving = getMovementStatus(obj)
            statusMsg = receive(obj.ROSinfo.controllerStateSub);
            velocities = statusMsg.Actual.Velocities;
            if all(velocities<0.1)
                isMoving = 0;
            else
                isMoving = 1;
            end
        end
        
        % Delete function
        function delete(obj)
            delete(obj.FlowChart)
        end
        
        function visualizePath(obj, positions)
            show(obj.Robot, obj.CurrentRobotJConfig,'PreservePlot', false, 'Frames', 'off');
            poses = zeros(size(positions,2),3);
            for i=1:size(positions,2)               
                poseNow = getTransform(obj.Robot, positions(:,i)', obj.RobotEndEffector);
                poses(i,:) = [poseNow(1,4), poseNow(2,4), poseNow(3,4)];
            end
            obj.PathHandle = plot3(poses(:,1), poses(:,2), poses(:,3),'r-','LineWidth',5);            
            drawnow;
        end
            
    end
  
end

