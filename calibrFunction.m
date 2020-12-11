function [worldPoints,camParam] = calibrFunction (files,squareSize)

% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);

% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.

I = imread(files{1});
imageSize = [size(I, 1), size(I, 2)];

camParam = estimateCameraParameters(imagePoints, worldPoints, ...
                                     'ImageSize', imageSize);
% Evaluate calibration accuracy.
figure; showReprojectionErrors(camParam);
title('Reprojection Errors');
return
