function Plot3DQuadRotor_with_Cam(Rr2n,Tn2r,Rc2r,Tr2c,r_size,c_size,r_color,c_color,r_zref,c_scale_ref)
%**************************************************************************  
% Rodrigo Munguia 2012
% Plots a 3D QuadRotor with a mounted camera
% Rr2n -> Robot to navigation rotation matrix [3x3]
% Tn2r -> Robot to navigation translation vector  [x y z]'
% Rc2r -> Camera to robot rotation matrix [3x3]
% Tr2c -> Camera to robot translation vector [x y z]'
% r_size -> size of the arm of the quadcopter
% c_size -> size of the camera
% r_color -> color of the quadrotor
% c_color -> color of the camera
% r_zref -> z reference for quad [x y z]', can be used for plotting the
% mount of the camera
% c_scale_ref -> scale size of the camera reference frame (respect to c_size) 
%
%**************************************************************************

 
     Plot3DQuadRotor(r_size,Rr2n,Tn2r,r_color,r_zref);
    
     % estimate the camera to navigation rotation matrix (Rc2n)
     Rc2n = Rr2n*Rc2r;  
     % estimate the navigation to camera translation vector (Tn2c)
     % having account the "arm effect"  (Rr2n*Tr2c)
     Tn2c =  Tn2r + Rr2n*Tr2c;  
   
     Plot3Dcam(c_size,c_scale_ref,Rc2n,Tn2c,c_color);
     
     alpha(0.5);
     
     
     