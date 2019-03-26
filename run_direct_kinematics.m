function run_direct_kinematics()
    %Run this program to compute the direct kinematics
    
    %set of 6 angles relative to each dof
    theta = [ 0 0 0 0 0 0 ];
    
    [n,s,a,p] = direct_kinematics(theta);
    disp([n s a p]);

end
