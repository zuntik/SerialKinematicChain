function [val] = check_rot_validity( r_matrix )

    check_sizes = norm(r_matrix(:,1))==1 && norm(r_matrix(:,2))==1 && norm(r_matrix(:,3))==1;
    check_othogonality = dot(r_matrix(:,1),r_matrix(:,2)) == 0 && dot(r_matrix(:,1),r_matrix(:,3)) == 0 && dot(r_matrix(:,2),r_matrix(:,3)) == 0;
    check_det = det(r_matrix)==1;
    val = (check_sizes == 1 && check_othogonality == 1 && check_det == 1);

end
