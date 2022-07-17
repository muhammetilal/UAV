function exampleCommandComputeGraspPoseROSGazeboScene(coordinator) 

%   Compute the grasp pose for a part with a known pose
%   This command computes the task-space grasping pose required for the
%   manipulator to pick up a part. The grasp pose is based on the pose of
%   the part to be picked up, which is saved as part.centerPoint during the object detection.
%
% Copyright 2020 The MathWorks, Inc.

        %   This functioncomputes the grasping pose by applying a
        %   pre-defined transform relative to the pose of the object. This
        %   could be replaced by more advanced methods. For example,
        %   the grasp pose could be found using point cloud data
        %   together with machine / deep learning based on object
        %   poses.
        nextPart = coordinator.Parts{coordinator.NextPart};
        if nextPart.type==2
            coordinator.GraspPose = trvec2tform(nextPart.centerPoint + [0 0 0.03])*axang2tform([0 1 0 pi]);
        else
            coordinator.GraspPose = trvec2tform(nextPart.centerPoint + [0 0 0.06])*axang2tform([0 1 0 pi]);
        end
end