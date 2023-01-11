function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%% BEFORE RUNNING THE CODE CHANGE NAME TO pred_step
    %% Parameter Definition
    % uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timestep
    %dt - difference in time 

%%    

    alpha = 0.001;
    k = 1;
    beta = 2;
    n = 15;
    nq = 12;
    n_dash = n + nq;
    lambda_dash = (alpha^2)*(n_dash+k) -n_dash;

    uAug = [uPrev; zeros(12,1)];   % 27 X 1
    pAug = [covarPrev zeros(15,12); zeros(12,15) eye(12,12)*0.01]; 
    pAug_root = chol(pAug,"lower");

    xAug = [];
    xdot = [];
    xt = [];

    for a = 0: 2*n_dash
        
        if (a == 0)

            x = uAug;

        elseif(a > 0 && a <= 27)

            x = uAug + (sqrt(n_dash + lambda_dash)*(pAug_root(:,a)));

        elseif(a > 27)
            x = uAug - (sqrt(n_dash + lambda_dash)*(pAug_root(:,a-n_dash)));

        end
        xAug = [xAug x];
        
    end

     % Euler rates


%    G = [ cos(uAug(6,1))*cos(uAug(5,1)), -sin(uAug(6,1)),   0;
%       sin(uAug(6,1))*cos(uAug(5,1)),  cos(uAug(6,1)),   0;
%       -sin(uAug(5,1)),               0,         1]; 

    % Rotation Matrix

% R = [cos(uAug(5,1))*cos(uAug(6,1)),      cos(uAug(6,1))*sin(uAug(4,1))*sin(uAug(5,1)) - cos(uAug(4,1))*sin(uAug(6,1)),      sin(uAug(4,1))*sin(uAug(6,1)) + cos(uAug(4,1))*cos(uAug(6,1))*sin(uAug(5,1));
%      cos(uAug(5,1))*sin(uAug(6,1)),      cos(uAug(4,1))*cos(uAug(6,1)) + sin(uAug(4,1))*sin(uAug(5,1))*sin(uAug(6,1)),      cos(uAug(4,1))*sin(uAug(5,1))*sin(uAug(6,1)) - cos(uAug(6,1))*sin(uAug(4,1));
%              -sin(uAug(5,1)),                                   cos(uAug(5,1))*sin(uAug(4,1)),                                                     cos(uAug(4,1))*cos(uAug(5,1))                               ];               

    %%
for i = 1:55
    % Euler rates

    G = [       -sin(xAug(5,i))                    0        1;
          cos(xAug(5,i))*sin(xAug(4,i))    cos(xAug(4,i))   0;
          cos(xAug(5,i))*cos(xAug(4,i))   -sin(xAug(4,i))   0];

    % Rotation Matrix

R = [cos(xAug(5,i))*cos(xAug(6,i)),      cos(xAug(6,i))*sin(xAug(4,i))*sin(xAug(5,i)) - cos(xAug(4,i))*sin(xAug(6,i)),      sin(xAug(4,i))*sin(xAug(6,i)) + cos(xAug(4,i))*cos(xAug(6,i))*sin(xAug(5,i));
     cos(xAug(5,i))*sin(xAug(6,i)),      cos(xAug(4,i))*cos(xAug(6,i)) + sin(xAug(4,i))*sin(xAug(5,i))*sin(xAug(6,i)),      cos(xAug(4,i))*sin(xAug(5,i))*sin(xAug(6,i)) - cos(xAug(6,i))*sin(xAug(4,i));
             -sin(xAug(5,i)),                                   cos(xAug(5,i))*sin(xAug(4,i)),                                                     cos(xAug(4,i))*cos(xAug(5,i))                               ];               
 

g = [0;  0; -9.81]; % gravity

wm = [angVel(1,1); angVel(2,1); angVel(3,1)]; % angular velocity

am = [acc(1,1); acc(2,1); acc(3,1)]; % acceleration

x3 = [xAug(7,i); xAug(8,i); xAug(9,i)]; % linear velocity

x4 = [xAug(10,i); xAug(11,i); xAug(12,i)]; % gyroscope bias

x5 = [xAug(13,i); xAug(14,i); xAug(15,i)]; % accelerometer bias

ng = [xAug(16,i); xAug(17,i); xAug(18,i)];

na = [xAug(19,i); xAug(20,i); xAug(21,i)];

nbg =[xAug(22,i); xAug(23,i); xAug(24,i)];

nba =[xAug(25,i); xAug(26,i); xAug(27,i)];

% Process Model

    Xdot = [x3; 
           inv(G)*R*(wm - x4 -ng);
           g+R*(am-x5-na);
           nbg;
           nba];

xdot = [xdot Xdot];


%xdot(:,i)
    Xt = (Xdot*dt) + xAug(1:15, i);

    xt = [xt Xt];

end



for j = 1: length(xt)

    if(j == 1)
        Wm = lambda_dash/(n_dash + lambda_dash);
        p = Wm*xt(:,j);
    else
        Wm = 1/(2*(n_dash + lambda_dash));
        p = p + (Wm*xt(:,j));
    end
end
uEst = p;

for j = 1: length(xt)

    if(j == 1)
        Wc = (lambda_dash/(n_dash + lambda_dash)) + (1 - alpha^2 + beta);
        q = Wc*(xt(:,j)-uEst)*transpose(xt(:,j)-uEst);
    else
        Wc =  1/(2*(n_dash + lambda_dash));
        q = q + (Wc*(xt(:,j)-uEst)*transpose(xt(:,j)-uEst));
    end

end
covarEst = q;

%%
end

