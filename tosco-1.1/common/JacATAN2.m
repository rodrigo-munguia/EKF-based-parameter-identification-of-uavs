function [da_dy da_dx] = JacATAN2(y,x)

%function [datan2_da datan2_db] = JacATAN2(a,b)


if x > 0
    
    a = atan(y/x);
    da_dxy =  [ -y/(x^2*(y^2/x^2 + 1)), 1/(x*(y^2/x^2 + 1))];
    da_dx = da_dxy(1);
    da_dy = da_dxy(2);
    
elseif (y >= 0)&&(x < 0 )
    
    
    a = atan(y/x) + pi;
    da_dxy =  [ -y/(x^2*(y^2/x^2 + 1)), 1/(x*(y^2/x^2 + 1))];
    da_dx = da_dxy(1);
    da_dy = da_dxy(2);
    
elseif (y < 0)&&(x < 0 )
    
    a = atan(y/x) - pi;
    da_dxy =  [ -y/(x^2*(y^2/x^2 + 1)), 1/(x*(y^2/x^2 + 1))];
    da_dx = da_dxy(1);
    da_dy = da_dxy(2);
    
elseif (y > 0)&&(x == 0)
    
    a = pi/2;
    da_dxy =  [ 0, 0];
    da_dx = da_dxy(1);
    da_dy = da_dxy(2);

elseif (y < 0)&&(x == 0)
    
    a = -pi/2;
    da_dxy =  [ 0, 0];
    da_dx = da_dxy(1);
    da_dy = da_dxy(2);
    
elseif (y == 0)&&(x == 0)    
    
    a = 0;
    da_dxy =  [ 0, 0];
    da_dx = da_dxy(1);
    da_dy = da_dxy(2);
end
    

