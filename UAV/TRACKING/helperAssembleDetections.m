function mydetections = helperAssembleDetections(bboxes,measNoise,timestamp)
% Assemble bounding boxes as cell array of objectDetection

mydetections = cell(size(bboxes,1),1);
for i = 1:size(bboxes,1)
    classid = bboxes(i,end);
    lidarModel = [bboxes(i,1:3), bboxes(i,end-1), bboxes(i,4:6)];
    % To avoid direct confirmation by the tracker, the ClassID is passed as
    % ObjectAttributes.
    mydetections{i} = objectDetection(timestamp, ...
        lidarModel','MeasurementNoise',...
        measNoise,'ObjectAttributes',struct('ClassID',classid));
end
end