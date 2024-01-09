function [q_a2b] = Ra2b_TO_Quat_a2b(Ra2b)

%   R_n_to_b  to Quaternion b  %D.15
R_n_to_b = Ra2b;

b1 = (1/2)*sqrt( 1 + R_n_to_b(1,1)+ R_n_to_b(2,2)+ R_n_to_b(3,3) );
     
q_a2b = [   b1;
        (R_n_to_b(3,2)-R_n_to_b(2,3))/(4*b1);
        (R_n_to_b(1,3)-R_n_to_b(3,1))/(4*b1);
        (R_n_to_b(2,1)-R_n_to_b(1,2))/(4*b1)]; % D.15
% *************************