function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    
    %% Output Parameter Description
    for i = 1: length(id)
        
        a = id(1,i);

        if (a <= 35)
            x0 = 0.152/2 + 0.152*2*rem(a,12);
            y0 = 0.152/2 + 0.152*2*fix(a/12);
        elseif(a>35 && a<72)
            x0 = 0.152/2 + 0.152*2*rem(a,12);
            y0 = 0.152/2 + 0.152*2*fix(a/12) + 0.026;
        elseif (a >=72) 
            x0 = 0.152/2 + 0.152*2*rem(a,12);
            y0 = 0.152/2 + 0.152*2*fix(a/12) + 0.026*2;
        end

    x1 = x0 + 0.152/2;
    y1 = y0 - 0.152/2;
    x2 = x0 + 0.152/2;
    y2 = y0 + 0.152/2;
    x3 = x0 - 0.152/2;
    y3 = y0 + 0.152/2;
    x4 = x0 - 0.152/2;
    y4 = y0 - 0.152/2;
          
    res(:,i) = [x0; y0; x1; y1; x2; y2; x3; y3; x4; y4];
    end
end