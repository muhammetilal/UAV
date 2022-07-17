clc
clear
clear all
[ptClouds,pretrainedModel] = helperDownloadData;

new_data=linspace(0,60,60); % it is used for time

%% ikinci bir ley için deneme ikisini entegere etmeye çalışacapım
a=load('lidarPointClouds.mat');
new_data=a.lidarPointClouds;
ishow(new_data);
%%
% Load point cloud
ptCloud = ptClouds{1};
% Define ROI for cropping point cloud 
yLimit = [-12,12];
zLimit = [-3,15];

roi = [xLimit,yLimit,zLimit];
% Extract ground plane
[nonGround,ground] = helperExtractGround(ptCloud,roi);
figure;
pcshowpair(nonGround,ground);
legend({'\color{white} Nonground','\color{white} Ground'},'Location','northeastoutside');
%% trying new data
% ikinci_data = load("lidarPointClouds.mat");
% lidarPointClouds = data.lidarPointClouds;
%%
% Load and visualize a sample frame
frame = helperPrepareData(ptCloud);
figure;
subplot(5,1,1);
imagesc(frame(:,:,1));
title('X channel');

subplot(5,1,2);
imagesc(frame(:,:,2));
title('Y channel');

subplot(5,1,3);
imagesc(frame(:,:,3));
title('Z channel');

subplot(5,1,4);
imagesc(frame(:,:,4));
title('Intensity channel');

subplot(5,1,5);
imagesc(frame(:,:,5));
title('Range channel');
%%
if ~exist('net','var')
    net = pretrainedModel.net;
end

% Define classes
classes = ["background","car","truck"];


%the kcolor map is changed
% Define color map
lidarColorMap = [
            0.98  0.98   0.00  % unknown
            0.01  0.98   0.01  % green color for car
            0.01  0.01   0.98  % blue color for motorcycle
            ];

% Run forward pass
pxdsResults = semanticseg(frame,net);

% Overlay intensity image with segmented output
segmentedImage = labeloverlay(uint8(frame(:,:,4)),pxdsResults,'Colormap',lidarColorMap,'Transparency',0.5);

% Display results
figure;
imshow(segmentedImage);
helperPixelLabelColorbar(lidarColorMap,classes);
%% kamyonu bulan kod
%burda başka bir şey detect etmem gerekiyor
truckIndices = pxdsResults == 'truck';
truckPointCloud = select(nonGround,truckIndices,'OutputSize','full');

% Crop point cloud for better display
croppedPtCloud = select(ptCloud,findPointsInROI(ptCloud,roi));
croppedTruckPtCloud = select(truckPointCloud,findPointsInROI(truckPointCloud,roi));

% Display ground and nonground points
figure;
pcshowpair(croppedPtCloud,croppedTruckPtCloud);
legend({'\color{white} Nonvehicle','\color{white} Vehicle'},'Location','northeastoutside');
%% truck detection
[labels,numClusters] = pcsegdist(croppedTruckPtCloud,1);

% Define cuboid parameters
params = zeros(0,9);

for clusterIndex = 1:numClusters
    ptsInCluster = labels == clusterIndex;
        
    pc = select(croppedTruckPtCloud,ptsInCluster);
    location = pc.Location;
    
    xl = (max(location(:,1)) - min(location(:,1)));
    yl = (max(location(:,2)) - min(location(:,2)));
    zl = (max(location(:,3)) - min(location(:,3)));
    
    % Filter small bounding boxes
    if size(location,1)*size(location,2) > 20 && any(any(pc.Location)) && xl > 1 && yl > 1
        indices = zeros(0,1);
        objectPtCloud = pointCloud(location);        
        for i = 1:size(location,1)
            seedPoint = location(i,:);
            indices(end+1) = findNearestNeighbors(nonGround,seedPoint,1);
        end
        
        % Remove overlapping indices        
        indices = unique(indices);
        
        % Fit oriented bounding box
        model = pcfitcuboid(select(nonGround,indices));
        params(end+1,:) = model.Parameters;
    end
end


% Display point cloud and detected bounding box
figure;
pcshow(croppedPtCloud.Location,croppedPtCloud.Location(:,3));
showShape('cuboid',params,"Color","red","Label","Truck");
%%
display = helperLidarObjectDetectionDisplay;
%%
% Initialize lidar object detector
lidarDetector = helperLidarObjecDetector('Model',net,'XLimits',xLimit,...
    'YLimit',yLimit,'ZLimit',zLimit);

% Prepare 5-D lidar data
inputData = helperPrepareData(ptClouds);

% Set random number generator for reproducible results
S = rng(2018);

% Initialize the display
initializeDisplay(display);

numFrames = numel(inputData);
for count = 1:numFrames
    
    % Get current data
    input = inputData{count};
    
    rangeImage = input(:,:,5);
    
    % Extact bounding boxes from lidar data
    [boundingBox,coloredPtCloud,pointLabels] = detectBbox(lidarDetector,input);
            
    % Update display with colored point cloud
    updatePointCloud(display,coloredPtCloud);
    
    % Update bounding boxes
    updateBoundingBox(display,boundingBox);
    
    % Update segmented image 
    updateSegmentedImage(display,pointLabels,rangeImage);
    
    drawnow('limitrate');
end
%%
assignmentGate = [10 100]; % Assignment threshold;
confThreshold = [7 10];    % Confirmation threshold for history logi
delThreshold = [2 3];     % Deletion threshold for history logic
Kc = 1e-5;                 % False-alarm rate per unit volume

% IMM filter initialization function
filterInitFcn = @helperMultiClassInitIMMFilter;

% A joint probabilistic data association tracker with IMM filter
tracker = trackerJPDA('FilterInitializationFcn',filterInitFcn,...
    'TrackLogic','History',...
    'AssignmentThreshold',assignmentGate,...
    'ClutterDensity',Kc,...
    'ConfirmationThreshold',confThreshold,...
    'DeletionThreshold',delThreshold,'InitializationThreshold',0);

allTracks = struct([]);
time = 0;
dt = 0.1;

% Define Measurement Noise
measNoise = blkdiag(0.25*eye(3),25,eye(3));

numTracks = zeros(numFrames,2);
%%
display = helperLidarObjectDetectionDisplay;
initializeDisplay(display);

for count = 1:numFrames
    time = time + dt;
    % Get current data
    input = inputData{count};
    
    rangeImage = input(:,:,5);
    
    % Extact bounding boxes from lidar data
    [boundingBox,coloredPtCloud,pointLabels] = detectBbox(lidarDetector,input);
    
    % Assemble bounding boxes into objectDetections
    detections = helperAssembleDetections(boundingBox,measNoise,time);
    
    % Pass detections to tracker
    if ~isempty(detections)
        % Update the tracker
         [confirmedTracks,tentativeTracks,allTracks,info] = tracker(detections,time);
         numTracks(count,1) = numel(confirmedTracks);
    end
    
    % Update display with colored point cloud
    updatePointCloud(display,coloredPtCloud);
            
    % Update segmented image 
    updateSegmentedImage(display,pointLabels,rangeImage);
    
    % Update the display if the tracks are not empty
     if ~isempty(confirmedTracks)
        updateTracks(display,confirmedTracks);
     end
     
    drawnow('limitrate');
end
%%
