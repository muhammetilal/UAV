function remainingParts = exampleCommandPickingLogicROSGazeboScene(coordinator)

% Determine which parts to pick next
%   This command instructs the robot which parts to pick next based on the
%   order of a preset list.
%
% Copyright 2020 The MathWorks, Inc.

       % Parts will be picked according to order in coordinator.detectedParts list
        coordinator.NextPart = coordinator.NextPart + 1; 
        if coordinator.NextPart<=length(coordinator.Parts)               
            % Objects are placed on either bin1 or bin2 according to
            % their type
            if coordinator.DetectedParts{coordinator.NextPart}.type == 1
                coordinator.DetectedParts{coordinator.NextPart}.placingBelt = 1;                    
            else
                coordinator.DetectedParts{coordinator.NextPart}.placingBelt = 2;
            end
            disp(coordinator.DetectedParts{coordinator.NextPart}.type)
            remainingParts = true; 
            return;
        end
        remainingParts = false; 
end