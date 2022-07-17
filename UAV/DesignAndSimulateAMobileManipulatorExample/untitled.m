clc;
clear;
clear all;
rosshutdown
ipaddress = "192.168.204.128";
rosinit(ipaddress)
%% robot initial position

%% joint ayarlama
physicsClient = rossvcclient('gazebo/unpause_physics');
physicsResp = call(physicsClient,'Timeout',3);
CommandActivateGripperROSGazebo('off');
[trajPub,trajCmd] = rospublisher('/husky_gen3/gen3_joint_trajectory_controller/command');

%jointWaypoints = [0 0 180 266 0 272 90]*pi/180; % bu jointleri ayarlıyor
jointWaypoints = [0 -10 180 275 0 277 100]*pi/180; % bu jointleri ayarlıyor

jointWaypointTimes = 1;

reachJointConfiguration(trajPub,trajCmd,jointWaypoints,jointWaypointTimes)
pause(1);

  temp = load('helperYolo2DetectorMobileArm.mat');
robotPubl = rospublisher("/husky_gen3/husky_velocity_controller/cmd_vel","DataFormat","struct") ;
cameraSub = rossubscriber("/camera/color/image_raw", "DataFormat", "struct");
cameraDep = rossubscriber("/camera/depth/points", "DataFormat", "struct");

%% object detection
for i=1:100
    

cammes = receive(cameraSub,1);
    image =rosReadImage(cammes);

    camdepmes = receive(cameraDep,1);
    %ss ten faydalamarak yaptım simlinkte, simulinkte yapılabilir

    xyz = rosReadXYZ(camdepmes);
points = reshape(xyz,480,270,3);
    
    %480 270 3
    
   
  DetectorModel = temp.detectorYolo2;   
     %  hfov = 1.211269;
     
     
     
      centerPixel = [round(size(image,1)/2), round(size(image,2)/2)];
     imshow(image);
     [bboxes,~,labels] = detect(DetectorModel,image);
     size_det=size(bboxes);
     if size_det(1,1)<1   
velo = rosmessage(robotPubl);
velo.Linear.X = 5;
send(robotPubl,velo)
     else
velo = rosmessage(robotPubl);
velo.Linear.X = 0
send(robotPubl,velo)
    break
     end
          pause(1);
     end
     
     
     load('exampleHelperKINOVAGen3GripperGazeboRRTScene.mat'); 

gen3 = robot;

% Retrive Camera properties
jSub = rossubscriber('/camera/color/camera_info');
jMsg = receive(jSub,1);

Widthimg = single(jMsg.Width);
Heightimg = single(jMsg.Height);
Lengthfoc = single(jMsg.K(1));

% End Effector
endEffectorFrame = "gripper";

% Read current Joint Angles
jSub = rossubscriber('/husky_gen3/gen3_joint_trajectory_controller/state');
jMsg = receive(jSub,1);
CRobotJConfig = wrapToPi(jMsg.Actual.Positions(1:7)');

% Get transformation to end effector
camTr = getTransform(gen3, CRobotJConfig, 'EndEffector_Link');
camZ = camTr(3,4);
zDist = camZ - 0.05590 + 0.10372; % - Width of the object + Width of the stool;
     
     
     %detecting function
     [objectResults,labeledImg] = detectObjects(bboxes, labels, points, image, camTr);
     imshow(labeledImg)
     %%
     %finding grasppose local 
% graspPoseLocal = computeGraspPose() 


% Approach detected object to pick from above
jSub = rossubscriber('/husky_gen3/gen3_joint_trajectory_controller/state');
jMsg = receive(jSub,1);
currentArmPose = wrapToPi(jMsg.Actual.Positions(1:7)');


taskActive = true;detectedObject=objectResults;
graspPoseLocal = computeGraspPose(detectedObject);
graspPose = graspPoseLocal;  % compute grasp pose based on the location of detected object

graspPose=graspPose(1:4,1:4);





 %% mapping
 
 
 load("helperRecyclingWarehouseMap.mat");
show(logicalMap)
 %%


%% robot position
odomSub = rossubscriber("/gazebo/model_states","DataFormat","struct");
odomMsg = receive(odomSub,3);

% pose = odomMsg.Pose.Pose;
X = odomMsg.Pose(16).Position.X;
Y = odomMsg.Pose(16).Position.Y;
z = odomMsg.Pose(16).Position.Z;


% X = model_states.Pose(huskyIdx).Position.X;
% Y = model_states.Pose(huskyIdx).Position.Y;
Xo = odomMsg.Pose(16).Orientation.X;
Yo = odomMsg.Pose(16).Orientation.Y;
Zo = odomMsg.Pose(16).Orientation.Z;
Wo = odomMsg.Pose(16).Orientation.W;

eul = quat2eul([Wo,Xo,Yo,Zo]);
yaw = eul(1);

position = [X,Y,yaw]; 
%position=[0 0 yaw];





% X = model_states.Pose(huskyIdx).Position.X;
% Y = model_states.Pose(huskyIdx).Position.Y;
Xr = odomMsg.Pose(12).Orientation.X;
Yr = odomMsg.Pose(12).Orientation.Y;
Zr = odomMsg.Pose(12).Orientation.Z;
Wr = odomMsg.Pose(12).Orientation.W;

eulr = quat2eul([Wr,Xr,Yr,Zr]);
yawr = eulr(1);


Xr = odomMsg.Pose(12).Position.X;
Yr = odomMsg.Pose(12).Position.Y;
Zr = odomMsg.Pose(12).Position.Z;



r_position = [Xr,Yr,Zr]; %robot position % current robot position olarak kullan



jSub = rossubscriber('/camera/color/camera_info');
jMsg = receive(jSub,4);

Widthimg = single(jMsg.Width);
Heightimg = single(jMsg.Height);
Lengthfoc = single(jMsg.K(1));

% End Effector
endEffectorFrame = "gripper";

% Read current Joint Angles
jSub = rossubscriber('/husky_gen3/gen3_joint_trajectory_controller/state');
jMsg = receive(jSub,10);
CRobotJConfig = wrapToPi(jMsg.Actual.Positions(1:7)');

% Get transformation to end effector
camTr = getTransform(gen3, CRobotJConfig, 'EndEffector_Link');
camZ = camTr(3,4);
zDist = camZ - 0.05590 + 0.1037; % - Width of the object + Width of the stool;



% Read the object's position and orientation data.
lidarSub = rossubscriber("/husky_gen3/scan","DataFormat","struct");
scanMsg = receive(lidarSub,10);

[cart,angles] = rosReadCartesian(scanMsg);
% xyz = rosReadXYZ(scanMsg)
%rosPlot(scanMsg)
% 
% gzinit(ipaddress);
% modelList = gzmodel("list")

% [r_position,g_selfcollide] = gzmodel("get","Green Can","Position","SelfCollide")
% [g_position,r_selfcollide] = gzmodel("get","Red Bottle","Position","SelfCollide")
[deger angle]=min(scanMsg.Ranges);
angle=angle/2;
centerPixel = [Heightimg/2, Widthimg/2];
centerBox = [Heightimg-r_position(1) Widthimg-r_position(2)];
centerBoxwrtCenterPixel = centerBox - centerPixel; % in pixels
worldCenterBoxwrtCenterPixel = (zDist/Lengthfoc)*centerBoxwrtCenterPixel; % in meters
actualCameraTransf = camTr * trvec2tform([0, 0.041, 0.0]);
actualpartXY = actualCameraTransf(1:2,4)' + worldCenterBoxwrtCenterPixel;
part_centerPoint = [actualpartXY(1)-0.13,actualpartXY(2)-0.49,zDist+0.55];
% part_centerPoint = [actualpartXY(1)+0.5,actualpartXY(2)-0.9,zDistance+0.45];

%r_position(3)=zDistance;
%part_centerPoint=r_position;


ik = inverseKinematics('RigidBodyTree',gen3);
ik.SolverParameters.AllowRandomRestart = 0;

% Calculate Final Pose to grasp the object
GraspPose = trvec2tform(part_centerPoint + [-0.008 0 -zDist-0.055])*eul2tform([deg2rad(0) pi 0]);



% Calculate first position Grasppose - 0.15 in Z axes
taskFinal = double(GraspPose*trvec2tform([0,0,-0.15]));


jointWaypointTimes = 1;
jointWaypoints1 = ik(endEffectorFrame,taskFinal,[1 1 1 1 1 1],CRobotJConfig);

% Go to Position 1
reachJointConfiguration(trajPub,trajCmd,jointWaypoints1,jointWaypointTimes);
%% finding object position
jointWaypointsnew = [0 0 180 266 0 272 90]*pi/180; % Scan position

jointWaypointTimes = 1;

reachJointConfiguration(trajPub,trajCmd,jointWaypointsnew,jointWaypointTimes);


%% location object
 load('exampleHelperKINOVAGen3GripperGazeboRRTScene.mat'); 
% 
 gen3 = robot;

% Retrive Camera properties


% GraspPose =  trvec2tform(part_centerPoint + [0 0 -0.035])*axang2tform([0 1 0 pi]);
%taskFinal = GraspPose*trvec2tform([0,0,-0.08]);
% tF=taskFinal;
% t0 =trvec2tform(position);
% tInterval = [0 5];
% tvec = 1;

% [tfInterp, v1, a1] = transformtraj(t0,tF,tInterval,tvec);


% Calculate first position Grasppose - 0.15 in Z axes
%taskFinal = double(GraspPose*trvec2tform([0,0,-0.15]));



%taskFinal=r_position;
weights=[1 1 1 1 1 1];
% jointWaypoints1 = ik(endEffectorFrame,taskFinal,weights,CurrentRobotJConfig);
% % jointWaypoints1=jointWaypoints1;
% jointWaypointTimes = 5;
% 
% % Go to Position 1
% reachJointConfiguration(trajPub,trajCmd,jointWaypoints1,jointWaypointTimes);

CommandActivateGripperROSGazebo('off');
pause(1);
% pause(1);
% % CommandActivateGripperROSGazebo('on');
% 
% jointWaypointTimes = 5;
% reachJointConfiguration(trajPub,trajCmd,jointWaypoints,jointWaypointTimes);    


%%
% Read current Joint Angles
jSub = rossubscriber('/husky_gen3/gen3_joint_trajectory_controller/state');
jMsg = receive(jSub,10);
CRobotJConfig = wrapToPi(jMsg.Actual.Positions(1:7)');

jointWaypoints2 = ik(endEffectorFrame,double(GraspPose),[1 1 1 1 1 1],CRobotJConfig);
% jointWaypoints1=jointWaypoints1*pi/180;
jointWaypointTimes = 1;
reachJointConfiguration(trajPub,trajCmd,jointWaypoints2,jointWaypointTimes);
pause(1);
% CommandActivateGripperROSGazebo('off');
%%
CommandActivateGripperROSGazebo('on');
pause(1);
%% belt
jointWaypointTimes = 1;
reachJointConfiguration(trajPub,trajCmd,jointWaypoints,jointWaypointTimes);    

% %beltCmd = rossubscriber("/gazebo/link_states");
% beltCmd = rospublisher("/gazebo/link_states") ;
% %beltMsg = receive(beltCmd,10); % 21 21
% belt_velocity=5;
% beltMsg = rosmessage(beltCmd);
% beltMsg.Twist(22,1).Linear.X = belt_velocity;
% send(beltCmd,beltMsg)
% send(beltCmd,beltMsg)


%% moving to conteiner

lidarSub = rossubscriber("/husky_gen3/scan","DataFormat","struct");
robotPubl = rospublisher("/husky_gen3/husky_velocity_controller/cmd_vel","DataFormat","struct") ;
velo = rosmessage(robotPubl);
for k=1:100
 
scanMsg = receive(lidarSub,1);
if scanMsg.Ranges(360)>0.60

velo.Linear.X = 5
send(robotPubl,velo)
else
velo.Linear.X = 0
send(robotPubl,velo)
break
pause(1)
end
end

jointWaypoints_end = [20 -10 180 275 0 277 100]*pi/180; % bu jointleri ayarlıyor
jointWaypointTimes = 1;
reachJointConfiguration(trajPub,trajCmd,jointWaypoints_end,jointWaypointTimes);