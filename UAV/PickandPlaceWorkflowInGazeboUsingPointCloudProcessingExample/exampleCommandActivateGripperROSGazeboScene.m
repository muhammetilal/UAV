function exampleCommandActivateGripperROSGazeboScene(coordinator, state)% This class is for internal use and may be removed in a future release
%
% Command function to activate gripper  
%   This command activates the gripper by sending a ROS action command. In
%   addition, it adds the picked part geometry to the collision Rigid Body Tree and
%   removes it when the part is placed.

% Copyright 2020 The MathWorks, Inc.

        %   Based on the state, decide whether to activate or
        %   deactivate the gripper
       if strcmp(state,'on') == 1
           %% Activate gripper
            pause(1);
            nextPart = coordinator.Parts{coordinator.NextPart};
            % Add new picked part in collision checking
            partBody = getBody(coordinator.Robot,'pickedPart');
            if nextPart.type == 1
                addCollision(partBody,"mesh", 'exampleHelperBottlePoints.stl', trvec2tform([0 0 0.08])*axang2tform([1 0 0 pi]));
                addVisual(partBody,"mesh", 'exampleHelperBottlePoints.stl', trvec2tform([0 0 0.08])*axang2tform([1 0 0 pi]));
                % Update visualization
                coordinator.WorldPatchHandles{nextPart.index}.Visible = 'off';
                show(coordinator.Robot, coordinator.CurrentRobotJConfig,'PreservePlot', false, 'Frames', 'off');
                drawnow;
                % Remove this part from the obstacles
                coordinator.World(nextPart.index) = []; 
            else
                addCollision(partBody,"mesh", 'exampleHelperCanPoints.stl', trvec2tform([0 0 0.01])*axang2tform([1 0 0 pi]));
                addVisual(partBody,"mesh", 'exampleHelperCanPoints.stl', trvec2tform([0 0 0.01])*axang2tform([1 0 0 pi]));
                % Update visualization
                coordinator.WorldPatchHandles{nextPart.index}.Visible = 'off';
                show(coordinator.Robot, coordinator.CurrentRobotJConfig,'PreservePlot', false, 'Frames', 'off');
                drawnow;
                % Remove this part from the obstacles
                coordinator.World(nextPart.index) = []; 
            end            
            % Update indices of remaining detected parts
            for i = (coordinator.NextPart + 1):numel(coordinator.Parts)
                if coordinator.Parts{i}.index > nextPart.index 
                    coordinator.Parts{i}.index = coordinator.Parts{i}.index -1;
                end
            end            
            % Generate gripper command
            [gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');
            gripperCommand = rosmessage('control_msgs/GripperCommand');
            gripperCommand.Position = 0.04; % 0.04 fully closed, 0 fully open
            gripperCommand.MaxEffort = 500;
            gripGoal.Command = gripperCommand;            
            pause(1);            
            % Send command
            sendGoal(gripAct,gripGoal); 
            disp('Gripper closed...');
       else
           %% Deactivate gripper
            pause(1);
            % Remove placed part from collision checking
            partBody = getBody(coordinator.Robot,'pickedPart');
            clearCollision(partBody);
            clearVisual(partBody);
            % Update visualization
            show(coordinator.Robot, coordinator.CurrentRobotJConfig,'PreservePlot', false, 'Frames', 'off');
            drawnow;
            % Generate gripper command
            [gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');
            gripperCommand = rosmessage('control_msgs/GripperCommand');
            gripperCommand.Position = 0.0; % 0.04 fully closed, 0 fully open
            gripperCommand.MaxEffort = 500;
            gripGoal.Command = gripperCommand;            
            pause(1);            
            % Send command
            sendGoal(gripAct,gripGoal);
            disp('Gripper open...');
       end
       
       pause(2);
end