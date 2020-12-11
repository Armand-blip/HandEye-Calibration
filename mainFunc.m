
% poses=readtable('posesRobot.csv');
% poses=poses{:,:};
% save('poses.mat','poses')
% load('poses.mat');



path='C:\Users\Stage-2\Documents\BMIfocus\MatlabCode\realRobotimages';
squareSize=15; % in mm

imageDir = fullfile(path);
images = imageDatastore(imageDir);

[worldPoints,camParam] = calibrFunction (images.Files',squareSize);

for i=1:size(images.Files,1)
    [Tc(:,:,i),cameraPos(i,:)]= extrinsicFunc(worldPoints,images,camParam,i);
end

%% First way to find Tg(transformation from gripper to base)
% Tg=zeros(4,4,20);
% for i=1:size(poses,1)
% 
%     eul=poses(i,5:7);
%     rotmZYX(:,:,i) = rotationVectorToMatrix(eul);
%     Tg(:,:,i) = [rotmZYX(:,:,i),poses(i,1:3)';0,0,0,1];
%  
% end

%% The second way to find Tg(trannsformation from gripper to base),using Rodrigues formula
Tg = Tc; 
%%If the poses are in .txt format use
poses = importdata([path '/newposes.txt']);
%% If we want the Euler angles in degrees
% poses(:,4:6)=deg2rad(poses(:,4:6));
for i = 1 : size(poses,1)
    pos = poses( i, 2:end);
    orientation = rotationVectorToMatrix(pos(4:6));
    location = pos(1:3);
    Tg(1:3, 1:3, i) = orientation';
    Tg(1:3, 4,   i) = location';
    Tg(4, :, i) = [0 0 0 1];
end
% T = ReadYaml(data_our);
X = handEye(Tg, Tc); %% Lazax method
disp(X);

% %% Find Z from equation AX=ZB
for i=1:size(poses,1)
    Z(:,:,i)= X*Tg(:,:,i)/Tc(:,:,i);
end
%% Find a z small ,the coordinates of object relative to the base of the robot,using Z from AX=ZB
z=pose_base2world(Z);

