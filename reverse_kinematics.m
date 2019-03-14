%function [is_possible] = two_theta_1_test(position,a,d) 
%% tests if a second value for theta_0 is possible
%    end_ef = [ sqrt(sum(position(1)^2+position(2)^2)) position(3) ]
%    awkward_point = [ -a(1) d(1) ]
%    
%    % vec is end minus awkward point
%    vec = end_ef - awkward_point;
%    straight_arm = sqrt( d(2) + sqrt(d()^2+a()^2) + a(5) );
%
%    if norm(vec) > straight_arm
%        is_possible = true;
%    else
%        is_possible = false;
%    end
%
%end



function [theta] = reverse_kinematics( position, r_matrix)
%reverse_kinematics Calculates the theta values for a position of the robot arm
%of the lab
% position must be column vector
% r_matrix is 3 x 3 matrix
% theta is a n x 6 matrix where n is the number of possible solutions

    if nargin < 2
        position = [ 1 1];
        r_matrix = eye(3);
    end

    % all values in cm
    a =     [   3     12   2       0       3       0   ];          
    d =     [   9.9   0     0       13      0       0   ];
    alpha = [   -pi/2 0     -pi/2   pi/2    pi/2    0   ];
    
    % p_w is the position of the base of the joint of the spherical wrist
    p_w = position - a(5) * r_matrix(:,3);

    % now we solve the two-link planar arm problem
    %   this will give the values for theta 2 and theta 3


    sols1 = planar_arm_sol([norm(p_w(1:2)) p_w(3)]-[a(1) d(1)],a(2),norm([a(3) d(4)]))
    
    % if the following function return empty array it's because the solution
    % is impossible, this is because the arms will not be long enough
    sols2 = planar_arm_sol([norm(p_w(1:2)) p_w(3)]-[-a(1) d(1)],a(2),norm([a(3) d(4)]))

    % TODO convert the angles of the planar_arm_sol func to angles we want
    
    sols1

    % TODO calculate rotation matrix and position of the 4th join for the,
    % potentially 4 solutions of the planar_arm_sol

    % if z of the 6th join is paralel to the z of the 4th joint then there are
    % infinite solutions but only one will be presented: theta(6)=theta(4)=0 
    
    theta_1(1) =  atan2(p_w(1),p_w(2));
    
    if true % return of planar_arm_sol not empty
        theta_1(2) =  atan2(-p_w(1),-p_w(2));
    end


end

