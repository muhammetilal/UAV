function multiChannelData = helperPrepareData(input)
% Create 5-channel data as x, y, z, intensity and range
% of size 64-by-1024-by-5 from pointCloud.

if isa(input, 'cell')
    numFrames = numel(input);
    multiChannelData = cell(1, numFrames);
    for i = 1:numFrames
        inputData = input{i};
        
        x = inputData.Location(:,:,1);
        y = inputData.Location(:,:,2);
        z = inputData.Location(:,:,3);
        
        intensity = inputData.Intensity;
        range = sqrt(x.^2 + y.^2 + z.^2);
        
        multiChannelData{i} = cat(3, x, y, z, intensity, range);
    end
else
    x = input.Location(:,:,1);
    y = input.Location(:,:,2);
    z = input.Location(:,:,3);
    
    intensity = input.Intensity;
    range = sqrt(x.^2 + y.^2 + z.^2);
    
    multiChannelData = cat(3, x, y, z, intensity, range);
end
end