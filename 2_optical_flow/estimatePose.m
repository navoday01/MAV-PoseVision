function [position, orientation, R_c2w] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    %% Output Parameter Defination
    
     X = getCorner(data(t).id);
    a = [];
    for i = 1:length(data(t).id)
        
        A = [X(1,i), X(2,i), 1, 0, 0, 0,  -data(t).p0(1,i)*X(1,i), -data(t).p0(1,i)*X(2,i),   -data(t).p0(1,i);
             0, 0, 0, X(1,i), X(2,i), 1,  -data(t).p0(2,i)*X(1,i), -data(t).p0(2,i)*X(2,i),   -data(t).p0(2,i);
             X(3,i), X(4,i), 1, 0, 0, 0,  -data(t).p1(1,i)*X(3,i), -data(t).p1(1,i)*X(4,i),   -data(t).p1(1,i);
             0, 0, 0, X(3,i), X(4,i), 1,  -data(t).p1(2,i)*X(3,i), -data(t).p1(2,i)*X(4,i),   -data(t).p1(2,i);
             X(5,i), X(6,i), 1, 0, 0, 0,  -data(t).p2(1,i)*X(5,i), -data(t).p2(1,i)*X(6,i),   -data(t).p2(1,i);
             0, 0, 0, X(5,i), X(6,i), 1,  -data(t).p2(2,i)*X(5,i), -data(t).p2(2,i)*X(6,i),   -data(t).p2(2,i);
             X(7,i), X(8,i), 1, 0, 0, 0,  -data(t).p3(1,i)*X(7,i), -data(t).p3(1,i)*X(8,i),   -data(t).p3(1,i);
             0, 0, 0, X(7,i), X(8,i), 1,  -data(t).p3(2,i)*X(7,i), -data(t).p3(2,i)*X(8,i),   -data(t).p3(2,i);
             X(9,i), X(10,i), 1, 0, 0, 0, -data(t).p4(1,i)*X(9,i), -data(t).p4(1,i)*X(10,i),  -data(t).p4(1,i);
             0, 0, 0, X(9,i), X(10,i), 1, -data(t).p4(2,i)*X(9,i), -data(t).p4(2,i)*X(10,i),  -data(t).p4(2,i)];
       
        a = [a;A]; 
    end
        [U_w,S_w,V_w] = svd(a); % using SVD

         h = reshape(V_w(:,9),[3,3]);
         
         h = transpose(h)/h(3,3);
         
         K = [311.0520        0        201.8724;
                0         311.3885    113.6210;
                0            0           1   ];  % Camera calibration matrix
        
         R_w = inv(K)*h;

         Mul = [R_w(:,1) R_w(:,2) cross(R_w(:,1),R_w(:,2))];

         [U,S,V] = svd(Mul); % using SVD

         R = U * [1 0 0; 0 1 0; 0 0 det(U*V')] * V'; % Rotation Matrix

         T = R_w(:,3)/norm(R_w(:,1)); % Translation Matrix

         Trf = [0.7071  -0.7071 0 0.04;
               -0.7071 -0.7071  0 0;
                0 0 -1 -0.03;
                0 0 0 1];                % Transformation of body with respect to camera

         C = [R; 0 0 0];
         D = [T; 1];

         Tbw = inv(Trf*[C D]);           % Transformation of body with respect to world

    position = Tbw(1:3,4);
    % translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order ZYX

    orientation = rotm2eul(Tbw(1:3,1:3),"ZYX");
    % euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order ZYX
    
    R_c2w = R';
    % Rotation which defines camera to world frame
end