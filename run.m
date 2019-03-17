%
% all values in cm
%theta = [ 0 -3*pi/4 pi/4 0 pi/2 0 ];
%theta = zeros(1,6);
theta = [  0 -3*pi/4 pi/4 0 0 ];

a =     [   3     12   2       0       0       0   ];          
d =     [   9.9   0     0       13      0       3   ];
alpha = [   -pi/2 0     -pi/2   pi/2    -pi/2    0   ];

[n,s,a,p] = direct_kinematics( theta, a, d, alpha );
disp(n);
disp(s);
disp(a);
disp(p);

