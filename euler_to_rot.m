function [rot] = euler_to_rot(Alpha, Beta, Gama, Mode)
    % mode is a 3 char string

    Z_mat = @(theta) [
        cos(theta) -sin(theta) 0
        sin(theta) cos(theta)  0
        0          0           1
    ];

    Y_mat = @(theta) [
        cos(theta)   0  sin(theta)
        0            1  0 
        -sin(theta)  0  cos(theta)
    ];
    
    X_mat = @(theta) [
        1   0            0
        0   cos(theta)   -sin(theta)
        0   sin(theta)   cos(theta)
    ];

    rot = eye(3);
    theta = [ Alpha, Beta, Gama ];

    Mode = lower(Mode);
    for i = 1:3
        if Mode(i)=='z'
            rot = rot * Z_mat(theta(i));
        end
        if Mode(i)=='x'
            rot = rot * X_mat(theta(i));
        end
        if Mode(i)=='y'
            rot = rot * Y_mat(theta(i));
        end
    end
    
end

