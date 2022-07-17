clc
clear
clear all
pcSeqFolder = fullfile(toolboxdir('driving'),'drivingdata','lidarSequence');
addpath(pcSeqFolder)
load timestamps.mat
rmpath(pcSeqFolder)
%%
%
%%
filename1='C:\Program Files\MATLAB\R2021b\toolbox\driving\drivingdata';
% fullfile(matlabroot,'C:\Program Files\MATLAB\R2021b\toolbox\driving\drivingdata')
cd(filename1)
groundTruthLabeler
%%
filename2="C:\Users\LENOVO\Desktop\UAV\ground_labeling";
cd(filename2);