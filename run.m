function run()
    
    % all values in cm
    %theta = [ 0 -3*pi/4 pi/4 0 pi/2 0 ];
    %theta = zeros(1,6);
    %theta = [  0 -3*pi/4 pi/4 pi/5 pi/6 pi/7 ];
    theta = [  0 -3*pi/4 pi/4 0 0 0 ];
    
    ai =     [ 3      12   2       0       0       0   ];
    di =     [ 9.9    0    0       13      0       3   ];
%    alphai = [ -pi/2  0    -pi/2   pi/2    -pi/2   0   ];
    alphai = [ -pi/2  0    -pi/2   -pi/2    pi/2   0   ];
    
    [n,s,a,p] = direct_kinematics( theta, ai, di, alphai );
    
    [theta] = reverse_kinematics(p, [n,s,a]);

    disp(theta);

    for i = 1:8
        [n,s,a,p] = direct_kinematics( theta(1,:), ai, di, alphai );
        disp([n s a p]);

    
%    disp(n);
%    disp(s);
%    disp(a);
%    disp(p);
%    disp(theta);
    
end
