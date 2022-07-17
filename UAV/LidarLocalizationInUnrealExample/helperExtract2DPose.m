function pose = helperExtract2DPose(tform)

eul = euler( quaternion(tform.Rotation', 'rotmat', 'point'), 'ZYX', 'point' );
pose = [tform.Translation(1), tform.Translation(2), eul(1)];
end