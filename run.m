function run()

    % all values in cm
    %theta = [  3*pi/4 -3*pi/4 pi/4 pi/5 pi/6 pi/7 ];

    theta = [ 0 0 pi/2 0 0 0 ];
    disp(theta*180/pi);

    ai =     [ 3      12   2       0       0      0   ];
    di =     [ 9.9    0    0       13      0      3   ];
    alphai = [ pi/2   0    pi/2   -pi/2    pi/2   0   ];
    
    [n,s,a,p] = direct_kinematics( theta, ai, di, alphai );
    disp([n s a p]);

    p = [ 25; 0; 11.9 ];
    rot = [
        0 0 -1 
        0 -1 0 
        -1 0 0
        ];

   [theta] = reverse_kinematics(p, rot);

   disp(round(theta*180/pi,2));

   disp(sprintf('there are %d solutions.',size(theta,1)));
   for i = 1:size(theta,1)
       [n,s,a,p] = direct_kinematics( theta(i,:), ai, di, alphai );
       disp([n s a p]);
   end

end
