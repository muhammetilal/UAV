function tform = helperPoseToRigidTransform(pose, angFormat)
%helperPoseToRigidTransform Convert pose to rigid transformation.
%   helperPoseToRigidTransform converts an M-by-6 matrix of poses specified
%   in the form [x, y, z, eulx, euly, eulz] to an M-by-1 vector of rigid3d
%   transformations. Euler angles are in the ZYX convention and assumed to
%   be in radians when angFormat is not specified.
%
%   See also rigid3d, quaternion

% Copyright 2020 The MathWorks, Inc.

if nargin==1
    angFormat = 'radians';
end

validateattributes(pose, {'single', 'double'}, {'2d', 'ncols', 6},...
    mfilename, 'pose');
angFormat = validatestring(angFormat, {'degrees', 'radians'}, mfilename, 'angFormat');

if isempty(pose)
    tform = rigid3d.empty;
    return;
end

numPoses = size(pose, 1);

trans  = pose(:, 1:3);
roteul = pose(:, 6:-1:4);

% Convert euler angles to quaternions
if angFormat(1)=='d'
    quat = quaternion(roteul, 'eulerd', 'ZYX', 'frame');
else
    quat = quaternion(roteul, 'euler', 'ZYX', 'frame');
end

% Convert quaternions to rotation matrices
rot = rotmat(quat, 'frame');

tform = repelem(rigid3d(eye(4, 'like', pose)), numPoses, 1);
for n = 1 : numPoses
    tform(n).Translation = trans(n, :);
    tform(n).Rotation    = rot(:, :, n);
end
end