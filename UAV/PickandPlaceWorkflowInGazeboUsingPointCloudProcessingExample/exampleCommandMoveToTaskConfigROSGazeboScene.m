function exampleCommandMoveToTaskConfigROSGazeboScene(coordinator, taskConfig)% This class is for internal use and may be removed in a future release
%
%   Move the manipulator to a task-space position
%   This command moves the manipulator from its current pose to a
%   desired task-space pose using RRT path planning. 

% Copyright 2020 The MathWorks, Inc.

    % First, check if robot is already at destination
    ik = inverseKinematics("RigidBodyTree", coordinator.Robot);
    ik.SolverParameters.AllowRandomRestart = false;
    ik.SolverParameters.EnforceJointLimits = false;
    warning('off','robotics:robotmanip:rigidbodytree:ConfigJointLimitsViolationAutoAdjusted');
    [goalConfig, solInfo] = ik(coordinator.RobotEndEffector, ...
    taskConfig, ones(1,6), ...
    coordinator.CurrentRobotJConfig);
    if solInfo.PoseErrorNorm>=0.01       
         disp("Warning: the Inverse Kinematic solver failed to converge.");
    end

    isAway = checkTargetAchieved();

    if isAway % if robot not at desired pose, plan a motion 
        %% Initialize RRT planner
        rng(2);                
        planner = manipulatorRRT(coordinator.Robot, coordinator.World);
        %% Compute a grasp configuration without collisions
        Tw_0 = taskConfig;
        Tw_0(1:3,1:3) = eye(3);
        Te_w = taskConfig;
        Te_w(1:3,4) = [0;0;0];
        bounds = [0, 0.0;
        0, 0;
        -0.0, 0.0;
        0, 0;
        0, 0;
        -0.001, 0.001;];
        Te_0 = taskConfig;
        searchiter=1;
        while(any(checkCollision(coordinator.Robot, goalConfig, coordinator.World)))
            Te_0 = exampleHelperSampleWGRROSGazeboScene(Te_w, Tw_0, bounds);
            [goalConfig, ~] = ik(coordinator.RobotEndEffector, ...
                Te_0, ones(1,6), ...
                coordinator.CurrentRobotJConfig);
            % disp("Searching for other config...",num2str(searchiter));
            searchiter = searchiter + 1;
        end
        warning('on','robotics:robotmanip:rigidbodytree:ConfigJointLimitsViolationAutoAdjusted');
        %% Plan a collision-free path
        startConfig = coordinator.CurrentRobotJConfig;
        planner.MaxConnectionDistance = 0.18;
        planner.ValidationDistance = 0.2;
        planner.EnableConnectHeuristic = true;
        disp("Now planning...");
        path = planner.plan(startConfig, goalConfig);
        % disp("Path planning is done.");
        %% Shorten path
        shortenedPath = shorten(planner, path, 15);
        %% Interpolate path
        planner.ValidationDistance = 0.02;
        positions = interpolate(planner, shortenedPath);
        positions = [coordinator.CurrentRobotJConfig;positions];
        robotPos = positions';
        %% Generate a trapezoidal trajectory from path
        h = 0.03;
        timeInterval = [0;h*size(positions,1)-h];
        [~,sd,~,~,~] = trapveltraj(timeInterval',size(positions,1));
        timeSteps = ((-sd+max(sd))/max(sd))*3*h+h;
        trajTimes = [0,cumsum(timeSteps(1:end-1))];
        %  Compute joint velocities and accelerations at required rate for execution by the robot
        % disp('Done planning trajectory, now sampling...')

        % Interpolated joint velocities
        robotVelTemp = wrapToPi(diff(robotPos'))./diff(trajTimes)';
        robotVel= [zeros(1,coordinator.NumJoints);robotVelTemp];

        % Interpolated joint accelerations
        robotAccTemp = diff(robotVel)./diff(trajTimes)';
        robotAcc = [zeros(2,coordinator.NumJoints);robotAccTemp];

        q = robotPos;
        qd = robotVel';
        qdd = robotAcc'; 
        
        %% Visualize path
        hold on
        visualizePath(coordinator,q);        
      
        %% Package and send the desired trajectory to robot for execution by the JointTrajectoryController
        %disp('Done sampling trajectory, now packaging...')
        [trajAct,trajGoal] = rosactionclient('/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory');
        trajAct.FeedbackFcn = [];
        exampleHelperPackageJointTrajectoryKINOVAROSGazeboScene(trajGoal,coordinator.ROSinfo.gazeboJointNames,q,qd,qdd,trajTimes);  
        waitForServer(trajAct);
        
        %disp('Done packaging trajectory, now sending...')
        sendGoal(trajAct,trajGoal)

        % Wait until the robot reaches destination
        pause(1.0);        % this pause is needed to start asking for movement status
        isAway = true;
        disp('Waiting until robot reaches the desired configuration');
        while isAway
            isAway = checkTargetAchieved();
            pause(1);
        end   

        % Wait until the robot stops moving again
        isMoving = true;
        % disp('Waiting until robot stops moving');
        while isMoving
            [isMoving] = getMovementStatus(coordinator);
            pause(1);
        end

    end

    % Update current robot configuration
    coordinator.CurrentRobotJConfig = getCurrentRobotJConfig(coordinator);
    coordinator.CurrentRobotTaskConfig = getTransform(coordinator.Robot, wrapToPi(coordinator.CurrentRobotJConfig), coordinator.RobotEndEffector); 
    
    % Delete path on plot after motion is complete
    hold on
    show(coordinator.Robot, coordinator.CurrentRobotJConfig,'PreservePlot', false, 'Frames', 'off');
    try
        coordinator.PathHandle.Visible = 'off';
    catch
        warning('Pathhandle was not found.');
    end

    function isAway = checkTargetAchieved()
        isAway = true;
        jointCurrent = wrapToPi(getCurrentRobotJConfig(coordinator));
        if all(abs(angdiff(jointCurrent,wrapToPi(goalConfig)))<0.15)     
            isAway=false; % goal achieved       
        end           
    end
end