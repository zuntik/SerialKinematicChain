function [sol1, sol2] = PlanarArm( a1, a2, pwx, pwy )
% Inverse Kinematics of a 3-link Planar Arm
% For a 3-link Planar Arm there are two different solutions

    c2 = (pwx^2+pwy^2-a1^2-a2^2)/(2*a1*a2);
    s2 = [sqrt(1-c2^2), -sqrt(1-c2^2)]; % two solutions for s2
    teta_2 = [atan2(s2(1), c2), atan2(s2(2), c2)]; % two solutions for teta_2
    
    s1 = [((a1+a2*c2)*pwy-a2*s2(1)*pwx)/(pwx^2+pwy^2),((a1+a2*c2)*pwy-a2*s2(2)*pwx)/(pwx^2+pwy^2)]; % two solutions for s1
    c1 = [((a1+a2*c2)*pwx+a2*s2(1)*pwy)/(pwx^2+pwy^2),((a1+a2*c2)*pwx+a2*s2(2)*pwy)/(pwx^2+pwy^2)]; % two solutions for s1
    teta_1 = [atan2(s1(1), c1(1)), atan2(s1(2), c1(2))]; % two solutions for teta_1
    % two solutions for the planar arm
    sol1 = [teta_1(1), teta_2(1)]; 
    sol2 = [teta_1(2), teta_2(2)];
end

