clear; % Clear variables
addpath('../data')
datasetNum = 1; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime,proj2Data] = init(datasetNum);

Z = sampledVicon(1:6,:);
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = 0.1*eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0; %last time step in real time
pos = proj2Data.position;
pose = proj2Data.angle;
%%
for i = 1:length(sampledTime)
    %% Fill in the FOR LOOP
    if(i == 1)
        oldt = prevTime;
    end
    newt = sampledData(1,i).t;
    dt = newt - oldt;
    angVel = sampledData(1,i).omg;
    acc = sampledData(1,i).acc;
    [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt);
    oldt = newt;
    z_t = [transpose(pos(i,:));transpose(pose(i,:))];
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
    uPrev = uCurr;
    covarPrev = covar_curr;
    savedStates(:,i) = uCurr;
    fprintf('%d\n',i)
end

plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);