function [theta] = reverse_kinematics( position, r_matrix )
%reverse_kinematics Calculates the theta values for a position of the robot arm
%of the lab
% position must be column vector
% r_matrix is 3 x 3 matrix
% theta is a n x 6 matrix where n is the number of possible solutions

    % all values in cm
    ai =     [ 3     12   2      0       0       0   ];
    di =     [ 9.9   0    0      13      0       3   ];
    alphai = [ pi/2  0    pi/2   -pi/2   pi/2    0   ];
    
    % p_w is the position of the base of the joint of the spherical wrist
    p_w = position - di(6) * r_matrix(:,3);

    if p_w(2) == 0 && p_w(1) == 0
        theta1(1) = 0;        
        if sum(abs(rot(:,3)-[ 0 0 1 ].'))<3*10e-5
            disp('Here we have infinite solutions');
        end
    else
        theta1(1) = atan2(p_w(2),p_w(1));
    end
    theta1(2) = rem(theta1(1) + pi, 2*pi );

    % now we solve the two-link planar arm problem
    %   this will give the values for theta 2 and theta 3
 
    planar_sols1 = planar_arm_sol([norm(p_w(1:2)) p_w(3)]-[ai(1) di(1)],ai(2),norm([ai(3) di(4)]),0);
    planar_sols2 = planar_arm_sol([norm(p_w(1:2)) p_w(3)]-[-ai(1) di(1)],ai(2),norm([ai(3) di(4)]),0);

    if isempty(planar_sols1) && isempty(planar_sols2)
        dips('No possible solutions');
        theta=[];
        return
    end
    
    if isempty(planar_sols2)
        two_t1 = 0;
    else
        two_t1 = 1;
    end
    
    adaptation_angle = atan(di(4)/ai(3));
    
    theta2(1:2,1) = planar_sols1(:,1);
    theta3(1:2,1) = planar_sols1(:,2) + adaptation_angle;
    if two_t1
        theta2(3:4,1) = pi-planar_sols2(:,1);
        theta3(3:4,1) = -planar_sols2(:,2) + adaptation_angle;
    end
        
    % calculate rotation matrix and position of the 4th join for the,
    % potentially 4 solutions of the planar_arm_sol
    % this can be done by doing the forward kinematics till the point

    r3_0 = zeros(3,3,2+2*two_t1);

    for i = uint32(1:size(r3_0,3))
        j = idivide(i,2,'ceil');
        [n,s,a,p]=direct_kinematics([theta1(j) theta2(i) theta3(i)],ai,di,alphai);
        r3_0(:,:,i) = [n,s,a];
    end

    r6_3 = zeros(size(r3_0));
    for i = 1:size(r3_0,3)
        r6_3(:,:,i) = r3_0(:,:,i)^-1 * r_matrix;
    end 

    % pass rotation matrix R^6_3 to spherical_wrist_sol

    theta = zeros(4*two_t1,6);
    for i = 1:uint32(size(r6_3,3))
        spherical_sols = spherical_wrist_sol(r6_3(:,:,i));
        theta(i*2-1,4:6) = spherical_sols(1,:);
        theta(i*2,4:6) = spherical_sols(2,:);
        theta(i*2-1,1) = theta1(idivide(i,2,'ceil'));
        theta(i*2,1) = theta1(idivide(i,2,'ceil'));
    end
    
    for i = uint32(1:size(theta,1))
        j = idivide(i,2,'ceil');
        theta(i,2:3) = [theta2(j) theta3(j)];
    end

end
