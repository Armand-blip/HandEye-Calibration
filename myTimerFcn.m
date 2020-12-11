
% this is used in app designer 
function myTimerFcn(app,~,~)
             
             cd('C:\Users\Stage-2\Desktop\HandEyeApp\Variables\hand-eye');
             if exist ("posesRobot.csv")
  
                imageDir = fullfile('C:\Users\Stage-2\Desktop\HandEyeApp\Images\realRobotImages'); 
                images = imageDatastore(imageDir);
                [worldPoints,cameraParams] = calibrFunction (images.Files',12);
                for i=1:size(images.Files,1)
                    [Tc(:,:,i),cameraPos(i,:)]= extrinsicFunc(worldPoints,images,cameraParams,i);
                end
%                 save(strcat(app.PathoutputEditField_HandEye.Value,'\Tc.mat'),'Tc') ;
                
                
                
                
                filename='posesRobot.csv'
                posesRobot=load(filename)
                for i=1:size(posesRobot,1)
                
                eul=posesRobot(i,1:3);
                rotmZYX(:,:,i) = eul2rotm(eul);
                Tg(:,:,i) = [rotmZYX(:,:,i),posesRobot(i,4:6)';0,0,0,1];
                end
                
                if ~exist("X.mat")
                X = handEye(Tc,Tg);
                save(strcat(app.PathoutputEditField_HandEye.Value,'\X.mat'),'X') ;
                else
                    logger(app,'File X.mat is already in our directory');
                end
                for i=1:size(posesRobot,1)
                    Z(:,:,i)= Tg(:,:,i)*X/Tc(:,:,i);
                end
                save(strcat(app.PathoutputEditField_HandEye.Value,'\Z.mat'),'Z') ;
                cd(app.PathoutputEditField_HandEye.Value);
                z=pose_base2world(Z);
                csvwrite('z.csv',z);
                 
             else
                 
                 logger(app, 'WARNING:The file of Robot poses is empty!');
                 
             end
             
         end