function interpPoseTform = helperInterpolatePoseTform(poseTform, relPoseTform, t)
%helperInterpolatePose Interpolate pose
%   helperInterpolatePoseTform interpolates a pose by a relative pose given
%   an interpolation coefficient.
%
%   interpPoseTform = helperInterpolatePoseTform(poseTform, relPoseTform, t)
%   interpolates the pose poseTform by the relative pose relPoseTform using
%   the interpolation coefficient t. t must lie in 0 to 1.
%
%   See also quaternion/slerp, rigid3d.

validateattributes(poseTform, {'rigid3d'}, {'scalar'}, mfilename, 'poseTform');
validateattributes(relPoseTform, {'rigid3d'}, {'scalar'}, mfilename, 'relPoseTform');
validateattributes(t, {'single', 'double'}, {'scalar', '>=', 0, '<=', 1}, mfilename, 't');

% Extract translation and quaternion from current pose transform
currTrans = poseTform.Translation;
currQuat  = quaternion(poseTform.Rotation', 'rotmat', 'point');

% Compute final pose transform using full relative pose transform
nextPoseTform = rigid3d( relPoseTform.T * poseTform.T );

% Extract translation and quaternion from final pose transform
nextTrans = nextPoseTform.Translation;
nextQuat  = quaternion(nextPoseTform.Rotation', 'rotmat', 'point');

% Interpolate translations
interpTrans = (1-t)*currTrans + t*nextTrans;

% Interpolate quaternion using slerp
interpQuat = slerp(currQuat, nextQuat, t);

% Convert back to rigid3d object
interpPoseTform = rigid3d( rotmat(interpQuat, 'point')', interpTrans );

end