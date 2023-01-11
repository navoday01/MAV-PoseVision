function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow
    % optPos = Position of the features in the camera frame 
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    %e = 0.8; %RANSAC hyper parameter
     
    psuccess = 0.99; % probability of success

     M = 3; % number of points

     k = log(1-psuccess)/log(1-(e^M)); % number of iterations required

     
     Maxinliners = 0; % initializing maximul numers of inliers

     for i = 1: k
     inliners = 0;
     randPos = randperm(length(optPos),3); % Random values of position
     p1 = optPos(randPos(1,1),:);
     p2 = optPos(randPos(1,2),:);
     p3 = optPos(randPos(1,3),:);
     
     
     H1 = [-1/Z(randPos(1,1)) 0 p1(1,1)/Z(randPos(1,1))  p1(1,1)*p1(1,2) -(1+p1(1,1)^2) p1(1,2);
            0 -1/Z(randPos(1,1)) p1(1,2)/Z(randPos(1,1)) (1+p1(1,2)^2) -p1(1,1)*p1(1,2) -p1(1,1)];

     H2 = [-1/Z(randPos(1,2)) 0 p2(1,1)/Z(randPos(1,2))  p2(1,1)*p2(1,2) -(1+p2(1,1)^2) p2(1,2);
           0 -1/Z(randPos(1,2)) p2(1,2)/Z(randPos(1,2)) (1+p2(1,2)^2) -p2(1,1)*p2(1,2) -p2(1,1)];

     H3 = [-1/Z(randPos(1,3)) 0 p3(1,1)/Z(randPos(1,3))  p3(1,1)*p3(1,2) -(1+p3(1,1)^2) p3(1,2);
           0 -1/Z(randPos(1,3)) p3(1,2)/Z(randPos(1,3)) (1+p3(1,2)^2) -p3(1,1)*p3(1,2) -p3(1,1)];

     H = [H1; H2; H3]; % H Matrix

     Vopt = [optV(2*randPos(1,1) - 1); optV(2*randPos(1,1)); optV(2*randPos(1,2) - 1); optV(2*randPos(1,2)); optV(2*randPos(1,3) - 1); optV(2*randPos(1,3))]; % optical flow velocity

     V = pinv(H)*Vopt; % camera twist

     inliners = 0;

     for a = 1: length(optPos)

         Hi = [-1/Z(a) 0 optPos(a,1)/Z(a)  optPos(a,1)*optPos(a,2) -(1+optPos(a,1)^2) optPos(a,2);
              0 -1/Z(a) optPos(a,2)/Z(a) (1+optPos(a,2)^2) -optPos(a,1)*optPos(a,2) -optPos(a,1)];
         
         Pi = [optV(2*a - 1); optV(2*a)];

         delta = norm(Hi*V -Pi);

         if(delta<= 0.005) 

            inliners = inliners +1;

         end

     end
      
     if(inliners >= Maxinliners)

         Maxinliners = inliners;
         Vel = V;
     end
     end



     

     
    
    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    
end