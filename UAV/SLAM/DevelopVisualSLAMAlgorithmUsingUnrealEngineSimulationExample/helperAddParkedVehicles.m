function helperAddParkedVehicles(model, poses)
%helperAddParkedVehicles Add parked vehicles to the parking lot scene
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.
%
%   Copyright 2020 The MathWorks, Inc.

% Set random seed for reproducibility
rng(0);

parkedVehicleBlkName  = [model, '/Parked Vehicles'];

vehicleType = {'Hatchback', 'Muscle car', 'Sport utility vehicle', 'Sedan', ...
    'Small pickup truck'};

vehicleColor = {'Red', 'Orange', 'Yellow', 'Green', 'Blue', 'Black', ...
    'Silver', 'White'};

numTypes    = numel(vehicleType);
numColors   = numel(vehicleColor);
numVehicles = size(poses, 1);

vehicleBlk  = 'drivingsim3d/Simulation 3D Vehicle with Ground Following';
constBlk    = 'simulink/Sources/Constant';

% Randomly pick vehicle type and vehicle color 
typeIdx     = randi(numTypes, 1, numVehicles);
colorIdx    = randi(numColors, 1, numVehicles);

% Randomly flip the orientation of a vehicle
flipOrientation = randi([0, 1], numVehicles, 1);
poses(:, 3) = poses(:, 3) + 180*flipOrientation;

existingsBlks = find_system(parkedVehicleBlkName);

if numel(existingsBlks) > 1 % Parked vehicles added
    return
end

% Add parked vehicles to the scene

for i = 1:size(poses, 1)
    % Add vehicle block
    hVehicle = add_block(vehicleBlk, [parkedVehicleBlkName, '/vehicle', num2str(i, 4)]);
    
    % Set vehicle type
    set_param(hVehicle, 'PassVehMesh', vehicleType{typeIdx(i)});
    
    % Set vehicle color
    set_param(hVehicle, 'VehColor', vehicleColor{colorIdx(i)});
    
    % Set initial location and orientation
    set_param(hVehicle, 'InitialPos', mat2str([poses(i,1:2), 0], 4));
    set_param(hVehicle, 'InitialRot', mat2str([0, 0, poses(i, 3)], 4));
    
    % Specify the trajectory of the parked vehicles as constants
    inports  = get_param(hVehicle, 'PortHandles');

    hX = add_block(constBlk, [parkedVehicleBlkName, '/X', num2str(i)]);
    set_param(hX, 'Value', num2str(poses(i, 1), 4));
    outX = get_param(hX, 'PortHandles');
    add_line(parkedVehicleBlkName, outX.Outport(1), inports.Inport(1));
    
    hY = add_block(constBlk, [parkedVehicleBlkName, '/Y', num2str(i)]);
    set_param(hY, 'Value', num2str(poses(i, 2), 4));
    outY = get_param(hY, 'PortHandles');
    add_line(parkedVehicleBlkName, outY.Outport(1), inports.Inport(2));
    
    hT = add_block(constBlk, [parkedVehicleBlkName, '/T', num2str(i)]);
    set_param(hT, 'Value', num2str(poses(i, 3), 4));
    outT = get_param(hT, 'PortHandles');
    add_line(parkedVehicleBlkName, outT.Outport(1), inports.Inport(3));
end

Simulink.BlockDiagram.arrangeSystem(parkedVehicleBlkName);
set_param(parkedVehicleBlkName, 'Zoomfactor', '100');
end