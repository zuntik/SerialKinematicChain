function [sols] = planar_arm_sol( pos, a1, a2, alpha ) 
    % a1, a2 are the sizes of the arms
    % alpha is the angle of the end effector with referents to the x axis
    % sols is 2 x 3 matrix with 1 line per solution
    % there are 3 angles, the 3rd may not be necessary and in shows the angle
    % that the end effector joint has to have (if it exists), in order for the
    % angle of the end effector join w.r.t. the origin

    if nargin < 4
        pos = [ 3 4];
        a1 = 3;
        a2 = 3;
        alpha = 0;
    end


    % test if solution is possible
    if norm(pos) > a1 + a2
        sols = [];
        return;
    end;

    c2 = (pos(1)^2+pos(2)^2-a1^2-a2^2)/(2*a1*a2);
    s2 = [ -sqrt(1-c2^2) sqrt(1-c2^2) ];
    theta2 = acos((pos(1)^2+pos(2)^2-a1^2-a2^2)/(2*a1*a2));
    theta2(2) = -theta2(1)

    s1(1) = ((a1 + a2*c2)*pos(2) - a2*s2(1)*pos(1))/(norm(pos)^2);
    c1(1) = ((a1 + a2*c2)*pos(1) - a2*s2(1)*pos(2))/(norm(pos)^2);
    s1(2) = ((a1 + a2*c2)*pos(2) - a2*s2(2)*pos(1))/(norm(pos)^2);
    c1(2) = ((a1 + a2*c2)*pos(1) - a2*s2(2)*pos(2))/(norm(pos)^2);

    theta1(1) =  atan2(s1(1),c1(1));
    theta1(2) =  atan2(s1(2),c1(2));
    theta3 = alpha - theta1 - theta2;

    sols = vertcat(theta1,theta2,theta3).';

end;
