function [theta] = spherical_wrist_sol( rot )
% rot is the position of the end effector w.r.t. the base frame
% theta is 2 x 3 matrix, one line for each solution

    %if isequal(rot(:,3),[ 0 0 1 ].')
    if sum(abs(rot(:,3)-[ 0 0 1 ].'))<3*10e-5
        theta = zeros(2,3);
        disp('Here we have infinite solutions');
        theta(1,1) = atan2(rot(2,1),rot(1,1));
        theta(2,3) = atan2(rot(2,1),rot(1,1));
    elseif sum(abs(rot(:,3)-[ 0 0 -1 ].'))<3*10e-5
        theta = zeros(2,3);
        disp('Here we have infinite solutions');
        theta(1,1) = atan2(rot(2,1),rot(1,1));
        theta(2,3) = atan2(rot(2,1),rot(1,1));
        theta(1,2) = pi;
        theta(2,2) = pi;
    else
        theta(1,1) = atan2(rot(2,3),rot(1,3));
        theta(1,2) = atan2(norm(rot(1:2,3)),rot(3,3));
        theta(1,3) = atan2(rot(3,2),-rot(3,1));

        theta(2,1) = atan2(-rot(2,3),-rot(1,3));
        theta(2,2) = atan2(-norm(rot(1:2,3)),rot(3,3));
        theta(2,3) = atan2(-rot(3,2),rot(3,1));
    end
end
