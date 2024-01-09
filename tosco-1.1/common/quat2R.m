% convert a quaternion b to a rotation matrix
function [Rn2b]=quat2R(b)
if norm(b)~= 0,
    b = b/norm(b);
    B = b(1);
    Bv = zeros(3,1);
    Bv(:,1)= b(2:4);
    Bc = [0    -Bv(3) Bv(2)
         Bv(3)  0    -Bv(1)
        -Bv(2)  Bv(1) 0];
     Rn2b = (B*B-Bv'*Bv)*eye(3,3)+2*Bv*Bv'+2*B*Bc;
    % Rn2b1 = [(b(1)^2+b(2)^2-b(3)^2-b(4)^2) 2*(b(2)*b(3)-b(1)*b(4)) 2*(b(1)*b(3)+b(2)*b(4))
    %         2*(b(2)*b(3)+b(1)*b(4)) (b(1)^2-b(2)^2+b(3)^2-b(4)^2) 2*(b(3)*b(4)-b(1)*b(2))
    %         2*(b(2)*b(4)-b(1)*b(3)) 2*(b(1)*b(2)+b(3)*b(4))   b(1)^2-b(2)^2-b(3)^2+b(4)^2]
         
         q = 10;
else
    Rn2b = eye(3,3)    % fault condition
    error('Norm b = 0');
end