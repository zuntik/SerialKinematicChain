function run_reverse_kinematics()
    %Run this program to compute the reverse kinematics

    %set position of end effector
    p = [ 3; 0; 4 ];
    %set orientation of end effector
    rot = [
        0 0 -1 
        0 -1 0 
        -1 0 0
        ];

   [theta] = reverse_kinematics(p, rot);
   %prints all possible solutions in degrees
   disp(round(theta*180/pi,2));

   
   fprintf('there are %d solutions. \n',size(theta,1));
   
   %computation of the direct kinematics using the solutions obtained with
   %reverese kinematics just to check if everything works correctly
   for i = 1:size(theta,1)
       [n,s,a,p] = direct_kinematics( theta(i,:) );
       disp([n s a p]);
   end

end
