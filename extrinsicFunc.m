function [T,cameraPoses] = extrinsicFunc (worldpoints,images,cameraParam,numFile)

% Display one o\f the calibration images
magnification = 100;
I=readimage(images, 1);
% figure; imshow(I, 'InitialMagnification', magnification);
% title('One of the Calibration Images'); 

% imOrig = imread([ images.Folders{1,1} '\Im' int2str(numFile) '.jpg']);

imOrig = imread(images.Files{numFile});
    
[im, newOrigin] = undistortImage(imOrig, cameraParam, 'OutputView', 'full');

% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);

% Adjust the imagePoints so that they are expressed in the coordinate system
% used in the original image, before it was undistorted.  This adjustment
% makes it compatible with the cameraParameters object computed for the original image.
imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints

% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldpoints, cameraParam);

cameraPoses=[t,rotm2eul(R)];
T =[R';0,0,0];
t=[t';1];
T=[T,t];

return