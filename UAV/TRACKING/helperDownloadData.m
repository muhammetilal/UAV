function [lidarData, pretrainedModel] = helperDownloadData
outputFolder = fullfile(tempdir,'WPI');
url = 'https://ssd.mathworks.com/supportfiles/lidar/data/lidarSegmentationAndTrackingData.tar.gz';
lidarDataTarFile = fullfile(outputFolder,'lidarSegmentationAndTrackingData.tar.gz');
if ~exist(lidarDataTarFile,'file')
    mkdir(outputFolder);
    websave(lidarDataTarFile,url);
    untar(lidarDataTarFile,outputFolder);
end
% Check if tar.gz file is downloaded, but not uncompressed
if ~exist(fullfile(outputFolder,'WPI_LidarData.mat'),'file')
    untar(lidarDataTarFile,outputFolder);
end
% Load lidar data
data = load(fullfile(outputFolder,'highwayData.mat'));
lidarData = data.ptCloudData;

% Download pretrained model
url = 'https://ssd.mathworks.com/supportfiles/lidar/data/pretrainedPointSegModel.mat';
modelFile = fullfile(outputFolder,'pretrainedPointSegModel.mat');
if ~exist(modelFile,'file')
    websave(modelFile,url);
end
pretrainedModel = load(fullfile(outputFolder,'pretrainedPointSegModel.mat'));
end