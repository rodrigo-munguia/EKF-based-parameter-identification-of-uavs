function [uvd,duv_dPc] = Projection_Model(Pc,type,Jac,u_type,cam_parameters,dist)

%**************************************************************************
% R. Munguia 2015
% 
% Projection of a point "Pc" (defined in camera coordinates) to the image plane [u  v]' 
%
% [u v] are in pixel coordiantes
% Output parameters: 
% 1) [u v] (distorted) are in pixel coordiantes
%  2) duv_dPc = Jacobian
% Input parameters:
% 1) Pc,  Point [x y z]' in euclidean coordinates expressed in camera
% coordinate frame
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
% 6) dist, Don't Apply distortion model -> 0 , apply ->1;

if (type == 1)  % projective model
    
    fc = cam_parameters.fc;
    cc = cam_parameters.cc;
    alpha_c = cam_parameters.alpha_c;
    
    if Pc(3) < 0   % if the point is behind the camera
       uvd = [NaN NaN]';
    else   % if the point is in front of the camera
        KK = [[fc(1) alpha_c*fc(2) cc(1)];
              [ 0     fc(2)       cc(2)];
              [ 0       0           1  ]];

        phom = KK*Pc;
    
    u = phom(1)/phom(3);
    v = phom(2)/phom(3);
    uv = [u v]';
    
        if (dist == 1)
            [ud vd] = Distort_a_Point(uv,cam_parameters,u_type);
            uvd = [ud vd]';
        elseif (dist == 0)
            uvd = uv;
        end
    end
    
    
   if (Jac == 0) % without jacobian
       duv_dPc = [];
   elseif (Jac == 1) % with jacobian
      
      dh_uuud = JAC_Distort_a_Point(uvd,cam_parameters,u_type);
      Pc1 = Pc(1);
      Pc2 = Pc(2);
      Pc3 = Pc(3); 
      cc1 = cam_parameters.cc(1);
      cc2 = cam_parameters.cc(2);
      fc1 = cam_parameters.fc(1);
      fc2 = cam_parameters.fc(2);
      alpha_c = cam_parameters.alpha_c;
      dhuuvu_Pc = [[ fc1/Pc3, (alpha_c*fc2)/Pc3, cc1/Pc3 - (Pc3*cc1 + Pc1*fc1 + Pc2*alpha_c*fc2)/Pc3^2]
                     [ 0, fc2/Pc3, cc2/Pc3 - (Pc3*cc2 + Pc2*fc2)/Pc3^2]]; 
       
     duv_dPc = dh_uuud*dhuuvu_Pc;
   end

elseif (type == 2)  % Omnidirectional model
    
    % to be implemented.....
    
    
    

end