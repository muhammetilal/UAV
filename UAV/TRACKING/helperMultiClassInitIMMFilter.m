function filter = helperMultiClassInitIMMFilter(detection)
% This is a helper function and may be removed in a future release.
% This function initializes an IMM filter for tracking using lidar data.

% Copyright 2019 The MathWorks, Inc.

% Create individual filters for constant turn rate and constant velocity.
trackingfilters = cell(2,1);
trackingfilters{1} = initCTCuboidFilter(detection);
trackingfilters{2} = initCVCuboidFilter(detection);

% Construct IMM and inform it about switching between models.
filter = trackingIMM(trackingfilters,'ModelConversionFcn',@switchimmCuboid);
filter.MeasurementNoise = detection.MeasurementNoise;
filter.TransitionProbabilities = 0.9;

end


function x = switchimmCuboid(modelType1,x1,modelType2,x2)
% This function simply removes helper and Cuboid from the model names and
% uses the switchimm function for switching kinematic state. It appends the
% non-kinematic part after the switch.
modelType1 = strrep(modelType1,'Cuboid','');
modelType1 = strrep(modelType1,'helper','');
modelType2 = strrep(modelType2,'Cuboid','');
modelType2 = strrep(modelType2,'helper','');

isCov = ~isvector(x1);
n1 = size(x1,1);
n2 = size(x2,1);
if ~isCov
    kinematicPartX1 = x1(1:(n1-4));
    shapePartX1 = x1((n1-3):n1);
    kinematicPartX2 = x2(1:(n2-4));
else
    kinematicPartX1 = x1(1:(n1-4),1:(n1-4));
    shapePartX1 = x1((n1-3):n1,(n1-3):n1);
    kinematicPartX2 = x1(1:(n2-4),1:(n2-4));
end

kinematicPartX = switchimm(modelType1,kinematicPartX1,modelType2,kinematicPartX2);

if isCov
    x = blkdiag(kinematicPartX,shapePartX1);
else
    x = [kinematicPartX;shapePartX1];
end

end

function filter = initCVCuboidFilter(detection)
    % This function initializes a constant-velocity cuboid filter from a
    % detection report.
    % detection contains measurement with the following convention:
    % [x;y;z;l;w;h];
   
    posIndex = [1 3 5];
    velIndex = [2 4 6];
    dimIndex = [8 9 10];
    yawIndex = 7;
    
    meas = detection.Measurement;
    measCov = detection.MeasurementNoise;
    dataType = class(meas);
    [pos,posCov,dim,dimCov,yaw,yawCov] = multiClassInverseLidarModel(meas, measCov, detection.ObjectAttributes.ClassID);
    velCov = blkdiag(10,5,1);
    
    % Assemble state and state covariances
    state = zeros(10,1,dataType);
    state(posIndex) = pos;
    state(dimIndex) = dim;
    state(yawIndex) = yaw;
    cov = zeros(10,dataType);
    cov(posIndex,posIndex) = posCov;
    cov(dimIndex,dimIndex) = dimCov;
    cov(yawIndex,yawIndex) = yawCov;
    cov(velIndex,velIndex) = velCov;
    
    % processNoise. Acceleration and omega
    Q = eye(4);
    Q(4,4) = 10;
    
    % Use a UKF for capture non-linearity.
    filter = trackingUKF(@helperConstvelCuboid,@helperCvmeasCuboid,state,...
         'StateCovariance',cov,...
        'HasAdditiveProcessNoise',false,...
        'ProcessNoise',Q,'MeasurementNoise',detection.MeasurementNoise,...
        'Alpha',0.01);

    n = numel(detection.Measurement);
    setMeasurementSizes(filter,n,n);
end


function filter = initCTCuboidFilter(detection)
    % This function initializes a constant-turn rate cuboid filter from a
    % detection report.
    % detection contains measurement with the following convention:
    % [x;y;z;l;w;h];
    
    posIndex = [1 3 6];
    dimIndex = [9 10 11];
    yawIndex = 8;
    velIndex = [2 4 7];
    
    meas = detection.Measurement;
    measCov = detection.MeasurementNoise;
    dataType = class(meas);
    [pos,posCov,dim,dimCov,yaw,yawCov] = multiClassInverseLidarModel(meas,measCov, detection.ObjectAttributes.ClassID);
    velCov = blkdiag(10,5,1);
    
    % Assemble state and state covariances
    state = zeros(11,1,dataType);
    state(posIndex) = pos;
    state(dimIndex) = dim;
    state(yawIndex) = yaw;
    cov = 100*eye(11,dataType);
    cov(posIndex,posIndex) = posCov;
    cov(dimIndex,dimIndex) = dimCov;
    cov(yawIndex,yawIndex) = yawCov;
    cov(velIndex,velIndex) = velCov;
    
    cov(5,5) = 10; % omegaCov;
    % processNoise
    Q = eye(4);
    Q(3,3) = 2.5;
    
    % Use a UKF for capture non-linearity.
    filter = trackingUKF(@helperConstturnCuboid,@helperCtmeasCuboid,state,...
        'StateCovariance',cov,...
        'HasAdditiveProcessNoise',false,...
        'ProcessNoise',Q,...
        'MeasurementNoise',detection.MeasurementNoise,...
        'Alpha',0.01);
    
    n = numel(detection.Measurement);
    setMeasurementSizes(filter,n,n);
end

function [pos,posCov,dim,dimCov,yaw,yawCov] = multiClassInverseLidarModel(meas,measCov,class)
% Shrink rate.
s = 3/50;
sz = 2/50;

% x,y and z of measurement
x = meas(1,:);
y = meas(2,:);
z = meas(3,:);
yaw = meas(4,:);
[az,~,r] = cart2sph(x,y,z);

% Shift x and y position.
Lshrink = abs(s*r.*(cos(az)));
Wshrink = abs(s*r.*(sin(az)));
Hshrink = sz*r;

shiftX = Lshrink;
shiftY = Wshrink;
shiftZ = Hshrink;

x = x + sign(x).*shiftX/2;
y = y + sign(y).*shiftY/2;
z = z + sign(z).*shiftZ/2;

pos = [x;y;z];
posCov = measCov(1:3,1:3,:);

%yaw = zeros(1,numel(x),'like',x);
yawCov = ones(1,1,numel(x),'like',x);

% Dimensions are initialized for a standard passenger car with low
% uncertainty.
if class == 1
    dim = [4.7;1.8;1.4];
    dimCov = 0.001*eye(3);
else
    dim = [6;2.5;2];
    dimCov = 0.001*eye(3);
end
end





