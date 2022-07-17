function pose = helperRigidTransformToPose(tform, angFormat)
%helperRigidTransformToPose Convert rigid transformation to pose.
%   helperRigidTransformToPose converts an M-by-1 array of rigid3d
%   transformation objects into an M-by-6 matrix of poses specified in the
%   form [x, y, z, eulz, euly, eulx]. Euler angles are in the ZYX
%   convention and assumed to be in radians if angFormat is not specified.
%
%   See also rigid3d, quaternion

% Copyright 2020 The MathWorks, Inc.

if nargin==1
    angFormat = 'radians';
end

validateattributes(tform, {'rigid3d'}, {'column'}, mfilename, 'tform');
angFormat = validatestring(angFormat, {'degrees', 'radians'}, mfilename, 'angFormat');

if isempty(tform)
    pose = zeros(0, 6);
    return;
end

% Pre-allocate output
pose = zeros(numel(tform), 6, 'like', tform(1).T);

trans  = vertcat(tform.Translation);
rotmat = cat(3, tform.Rotation);

% Convert rotation matrices to quaternions
quat = quaternion(rotmat, 'rotmat', 'frame');

% Convert quaternions to euler angles
if angFormat(1)=='d'
    roteul = eulerd(quat, 'ZYX', 'frame');
else
    roteul = euler(quat, 'ZYX', 'frame');
end

pose(:, 1:3) = trans;
pose(:, 4:6) = roteul;
end