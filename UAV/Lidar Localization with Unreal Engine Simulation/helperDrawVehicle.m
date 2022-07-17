function helperDrawVehicle(axesHandle, tform, dims, varargin)

pose = helperRigidTransformToPose(tform, 'degrees');
center = pose(:, 1:3);
rots   = fliplr(pose(:, 4:6));

position = [center, repmat(dims, numel(tform), 1), rots];
position(3) = position(3) + dims(3)/2; % Transform to center
showShape('cuboid', position, 'Parent', axesHandle, varargin{:});
end