function [n,s,a,p] = direct_kinematics(theta, ai, di, alphai)
%direct_kinematics Calculates the direct kinematics for the robot arm of
%the lab

    function m = prev_to_aux ( thta, d_ )
        m =  [
            cos(thta)   -sin(thta)  0  0
            sin(thta)   cos(thta)   0  0
            0           0           1  d_
            0           0           0  1     ];
    end

    function m = aux_to_next ( alph , a_ ) 
        m = [ 
            1   0          0           0
            0   cos(alph)  -sin(alph)  0
            0   sin(alph)  cos(alph)   0
            0   0          0           1 ];
        m = round(m);
        m(1,4) = a_;
    end


    mat = eye(4);
    for i = 1:length(theta)
        mat = mat*prev_to_aux(theta(i),di(i))*aux_to_next(alphai(i),ai(i));
    end

    n = mat(1:3,1);
    s = mat(1:3,2);
    a = mat(1:3,3);
    p = mat(1:3,4);

end
