function [hc,dhc_duvd] = Inverse_Projection_Model(uvd,type,Jac,u_type,cam_parameters)
%**************************************************************************
% R. Munguia 2015
% 
% Inverse projection model hc = [hc1 hc2 hc3]' = h^(-1)([u v]')
%
% Output parameters: 
% 1) Return a vector "hc" pointing in the direction of the undistorted image
% point, expressed in the camera frame.
% 2) dhc_duv = Jacobian
% Input parameters:
% 1) uvd,  distorted pixel coordinates of the image point [ud vd]
% 2) type, projective model -> 1 , omnidirectional model -> 2
% 3) Jac,  without jacobian -> 0 , with jacobian -> 1
% 4) u_type, undistortion model: 1-> Bouguet , 2-> A.J. Davison 2004, 3-> ...
% J.M.M. Montiel 2006 ( see Undistort_a_Point function)
% 5) Camera parameters: 
% for projective camera:  kc = cam_parameters.distortions;
%                         cc = cam_parameters.cc;
%                         fc = cam_parameters.fc;
% for omnidirectional camera:  kc = cam_parameters.Omni.distortions;
%                              XI = cam_parameters.Omni.xi;
%                              gamma = cam_parameters.Omni.gamma;
%                              CC = cam_parameters.Omni.CC;
%                              alpha_c = cam_parameters.Omni.alpha_c;
%

if (type == 1)  % projective model
    
    fc = cam_parameters.fc;
    cc = cam_parameters.cc;
    
    [uu, vu] = Undistort_a_Point(uvd,cam_parameters,u_type);
    
    hc = [(uu - cc(1) )/fc(1)  (vu - cc(2))/fc(2) 1]';
    
   if (Jac == 0) % without jacobian
       dhc_duvd = [];
   elseif (Jac == 1) % with jacobian
      
       dhc_duuvu = [[ 1/fc(1),     0]
                    [     0, 1/fc(2)]
                    [     0,      0]];
       %  JAC_Undistort_a_Point
       duuvu_dudvd =  inv(JAC_Distort_a_Point([uu vu],cam_parameters,u_type));
     
       
       dhc_duvd = dhc_duuvu*duuvu_dudvd;
       
   end

elseif (type == 2)  % Omnidirectional model
    
    % to be implemented.....
    
    
    

end