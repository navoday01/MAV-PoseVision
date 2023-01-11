function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state

    alpha = 0.001;
    k = 1;
    beta = 2;
    n = 15;
    lambda = alpha^2*(n+k)-n;
    covarEstRoot = chol(covarEst,"lower");
    %Rt = normrnd(0,(0.01), [3,1]);
    Rt = 0.0001*eye(3);
    %Rt = [0 0.0001 0; 0 0.0001 0; 0 0 0.0001];

    

    R_bc = [0.707 -0.707 0; -0.707 -0.707 0; 0 0 -1]; %Rotation of camera wrt body frame
    R_cb = R_bc';
    t_bc = [-0.04; 0.0; -0.03];  % Translation of camera wrt body frame
    T_bc =   [R_bc t_bc;0 0 0 1];  % Transformation matrix of body frame wrt camera frame
    T_cb = (T_bc)^-1; % Transformation matrix of camera wrt body
    R_bc = T_bc(1:3,1:3);
    t_cb = T_cb(1:3,4);
    adj = [R_bc -R_bc*skew(T_bc(1:3,4));zeros(3,3) R_bc];
    
    Xt = [];

    for a = 0: 2*n
        
        if (a == 0)

            x = uEst;

        elseif(a > 0 && a <= n)

            x = uEst + (sqrt(n + lambda)*(covarEstRoot(:,a)));

        elseif(a > n)
            x = uEst - (sqrt(n + lambda)*(covarEstRoot(:,a-n)));

        end
        Xt = [Xt x];
        
    end

 
    Zt = [];

    for b = 1: length(Xt)

       X2 = [Xt(6, b) Xt(5,b) Xt(4, b)];
       %R_wb = (eul2rotm(X2))';
       R_wb=(rotz(X2(1))*roty(X2(2))*rotx(X2(3)))';

       z = R_bc*R_wb*[Xt(7:9,b)]-R_bc*skew(t_cb)*R_bc*z_t(4:6,1);

       Zt = [Zt z];

    end
p = [];
 for c = 1: length(Zt)

    if(c == 1)
        Wm = lambda/(n + lambda);
        p = Wm*Zt(:,c);
    else
        Wm = 1/(2*(n + lambda));
        p = p + (Wm*Zt(:,c));
    end
end
zut = p;

for d = 1: length(Zt)

    if(d == 1)
        Wi = lambda/(n + lambda)+ (1 - alpha^2 + beta);
        q = Wi*(Xt(:,d)-uEst)*transpose(Zt(:,d)-zut);
        r = Wi*(Zt(:,d)-zut)*transpose(Zt(:,d)-zut) + Rt;
    else
        Wi = 1/(2*(n + lambda));
        q = q + Wi*(Xt(:,d)-uEst)*transpose(Zt(:,d)-zut);
        r = r + Wi*(Zt(:,d)-zut)*transpose(Zt(:,d)-zut) + Rt;
    end

end
Ct = q;
St = r ;

Kt = Ct*inv(St);

uCurr = uEst + Kt*(z_t(1:3)- zut);

covar_curr = covarEst - Kt*St*transpose(Kt);

end

