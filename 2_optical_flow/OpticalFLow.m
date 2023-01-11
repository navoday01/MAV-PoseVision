%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')


%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;
enableRANSAC = 0; % RANSAC flag, put 1 to enable RANSAC

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION



C = [311.0520        0        201.8724;
 
         0         311.3885    113.6210;
 
         0            0           1   ]; % Camer Calibration Matrix

 t = zeros(length(sampledData),1);
    for n = 1:length(sampledData)
         t(n) = sampledData(n).t;
    end
    t = sgolayfilt(t,1,101);   % filtering the time

for n = 2:length(sampledData)
    %% Initalize Loop load images
    prevImg = sampledData(n-1).img; % obtaining image for previous frame
    currImg = sampledData(n).img;   % obtaining image for current frame

    %% Detect good points
    goodPointsMatrix = detectMinEigenFeatures(prevImg); % keypoints detected
    goodPointsMatrix = goodPointsMatrix.selectStrongest(100);  % selected strongest hundred
    %% Initalize the tracker to the last frame.
    pointTracker = vision.PointTracker('MaxBidirectionalError',1); % tracker initialized
    
    %% Find the location of the next points;
    initialize(pointTracker,goodPointsMatrix.Location,prevImg);

    [Points,validity] = pointTracker(currImg);
    goodPoints = [];
    points = [];

    for a = 1: length(goodPointsMatrix)
        matrix1 = inv(C)*[goodPointsMatrix.Location(a,1);goodPointsMatrix.Location(a,2); 1];
        goodPoints = [goodPoints; matrix1(1,1) matrix1(2,1)];

        matrix2 = inv(C)*[Points(a,1);Points(a,2); 1];
        points = [points; matrix2(1,1) matrix2(2,1)];
    end 


    %% Calculate velocity
    % Use a for loop
    v = [];

     
    for b = 1: length(goodPoints)
    
     V = [(points(b,1)-goodPoints(b,1))/(t(n) - t(n-1)), (points(b,2)-goodPoints(b,2))/(t(n) - t(n-1))];
     
     v = [v;V(1,1); V(1,2)]; % Computing optical flow velocity

    end
    
    %% Calculate Height
    [position, orientation, R_c2w] = estimatePose(sampledData, n-1);

    Trf = [0.7071  -0.7071  0 0.04;
           -0.7071 -0.7071  0 0;
              0       0     -1 -0.03;
              0       0      0 1];    % Transformation Matrix of body w.r.t camera
 
    rot = eul2rotm(orientation);
   
    z = [];

    for c=1:length(goodPoints)

        Z = position(3)/(dot([goodPoints(c,1);goodPoints(c,2);1],-1*R_c2w(3,:)));

        z=[z;Z]; % Depth 
    end   

    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC
    if(enableRANSAC == 1)

    [Vel] = velocityRANSAC(v,goodPoints,z,R_c2w,0.5); % Camera Twist from RANSAC

    end
    %% Thereshold outputs into a range.
    % Not necessary

    %%
    if(enableRANSAC == 0)

    fun = [];
        for i = 1: length(goodPoints)
         
            Z = z(i);

            Funct = [-1/Z 0 goodPoints(i,1)/Z  goodPoints(i,1)*goodPoints(i,2) -(1+goodPoints(i,1)^2) goodPoints(i,2);
                0 -1/Z goodPoints(i,2)/Z (1+goodPoints(i,2)^2) -goodPoints(i,1)*goodPoints(i,2) -goodPoints(i,1)];

            fun = [fun; Funct];

        end

    Vel = pinv(fun)*v; % Computed camera Twist

    end
    
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame
    
    Vel =[rot zeros(3);zeros(3) rot]*[Trf(1:3,1:3) -Trf(1:3,1:3)*skew(Trf(1:3,4));zeros(3) Trf(1:3,1:3)]*Vel;
    %% ADD SOME LOW PASS FILTER CODE
    % Not neceessary but recommended 
    %estimatedV(:,n) = Vel;
    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    estimatedV(:,n) =Vel; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end

% Different cases for filtering data

if(datasetNum == 1 && enableRANSAC == 0)
estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 21);
estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 57);
estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 13);
estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 3);
estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 3);
estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 3);

elseif(datasetNum == 1 && enableRANSAC == 1)
estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 15);
estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 17);
estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 13);
estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 3);
estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 3);
estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 3);

elseif(datasetNum == 4 && enableRANSAC == 0)
estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 27);
estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 27);
estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 13);
estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 3);
estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 3);
estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 3);

elseif(datasetNum == 4 && enableRANSAC == 1)
estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 19);
estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 15);
estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 13);
estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 3);
estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 3);
estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 3);

end

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)

