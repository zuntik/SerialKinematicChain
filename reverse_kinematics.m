function [theta] = reverse_kinematics( position, r_matrix )
%reverse_kinematics Calculates the theta values for a position of the robot arm
%of the lab
% position must be column vector
% r_matrix is 3 x 3 matrix
% theta is a n x 6 matrix where n is the number of possible solutions


    % all values in cm
    ai =     [   3     12   2       0       0       0   ];
    di =     [   9.9   0     0       13      0       3   ];
    alphai = [   -pi/2 0     -pi/2   -pi/2    pi/2    0   ];
    
    % p_w is the position of the base of the joint of the spherical wrist
    p_w = position - di(6) * r_matrix(:,3);

    theta1(1) = atan2(p_w(2),p_w(1));
    theta1(2) = rem(theta1(1) + pi, 2*pi );

    % now we solve the two-link planar arm problem
    %   this will give the values for theta 2 and theta 3
 
    planar_sols1 = planar_arm_sol([norm(p_w(1:2)) -p_w(3)]-[ai(1) -di(1)],ai(2),norm([ai(3) di(4)]),0);
    planar_sols2 = planar_arm_sol([norm(p_w(1:2)) -p_w(3)]-[-ai(1) -di(1)],ai(2),norm([ai(3) di(4)]),0);

    adaptation_angle = atan(di(4)/ai(3));
    
    if not(isempty(planar_sols1))
        theta2 = planar_sols1(:,1);
        theta3 = planar_sols1(:,2) - adaptation_angle;        
    end
    if not(isempty(planar_sols2))
        theta2_v2 = planar_sols2(:,1);
        theta3_v2 = planar_sols2(:,2) - adaptation_angle ;
    end
        
    % calculate rotation matrix and position of the 4th join for the,
    % potentially 4 solutions of the planar_arm_sol
    % this can be done by doing the forward kinematics till the point

    if not(isempty(planar_sols1))
        [n,s,a,p]=direct_kinematics([theta1(1) theta2(1) theta3(1)],ai,di,alphai);
        r3_0(:,:,1) = [n,s,a];
        [n,s,a,p]=direct_kinematics([theta1(1) theta2(2) theta3(2)],ai,di,alphai);
        r3_0(:,:,2) = [n,s,a];
    end
    if not(isempty(planar_sols2))
        [n,s,a,p]=direct_kinematics([theta1(2) theta2_v2(1) theta3_v2(1)],ai,di,alphai);
        r3_0(:,:,3) = [n,s,a];
        [n,s,a,p]=direct_kinematics([theta1(2) theta2_v2(2) theta3_v2(2)],ai,di,alphai);
        r3_0(:,:,4) = [n,s,a];
    end

    if isempty(planar_sols1) && isempty(planar_sols2)
       disp('error');
    end
    
    r6_3 = zeros(3,3,4);
    for i = 1:4
        r6_3(:,:,i) = r3_0(:,:,i)^-1 * r_matrix;
    end

    % pass rotation matrix R^6_3 to spherical_wrist_sol

    theta = zeros(8,6);
    
    
    for i = 1:4
        spherical_sols = spherical_wrist_sol(r6_3(:,:,i));
        theta(i*2-1,4:6) = spherical_sols(1,:);
        theta(i*2,4:6) = spherical_sols(2,:);
        theta(i,1) = theta1(1);
        theta(i+4,1) = theta1(2);
    end;

    theta(1,2:3) = [theta2(1) theta3(1)];
    theta(2,2:3) = [theta2(1) theta3(1)];
    theta(3,2:3) = [theta2(2) theta3(2)];
    theta(4,2:3) = [theta2(2) theta3(2)];
    theta(5,2:3) = [theta2_v2(1)  theta3_v2(1)];
    theta(6,2:3) = [theta2_v2(1)  theta3_v2(1)];
    theta(7,2:3) = [theta2_v2(2)  theta3_v2(2)];
    theta(8,2:3) = [theta2_v2(2)  theta3_v2(2)];

    
    % if z of the 6th join is paralel to the z of the 4th joint then there are
    % infinite solutions but only one will be presented: theta(6)=theta(4)=0 
    


end
