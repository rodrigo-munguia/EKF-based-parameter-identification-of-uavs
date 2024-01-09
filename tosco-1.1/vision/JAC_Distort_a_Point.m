function [dh_uuud] = JAC_Distort_a_Point(uvu,cam_parameters,method)

if method == 1
       %**********************************************************************************
       % Distortion model used in Bouguet’s Calibration Toolbox for MATLAB® (Bouguet 2010)        
       % http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html 
       % WITHOUT tangencial distortion!!
       
        k1 = cam_parameters.distortions(1);
        k2 = cam_parameters.distortions(2);
        cc1 = cam_parameters.cc(1);
        cc2 = cam_parameters.cc(2);
        fc1 = cam_parameters.fc(1);
        fc2 = cam_parameters.fc(2);
        uv = uvu(1);
        vu = uvu(2);
       
       
       % improve implementation of derivatives!!
       dhud_uuvu =  [ k1*((cc1 - uv)^2/fc1^2 + (cc2 - vu)^2/fc2^2) + k2*((cc1 - uv)^2/fc1^2 + (cc2 - vu)^2/fc2^2)^2 + ((k1*(2*cc1 - 2*uv))/fc1^2 + (2*k2*((cc1 - uv)^2/fc1^2 + (cc2 - vu)^2/fc2^2)*(2*cc1 - 2*uv))/fc1^2)*(cc1 - uv) + 1, ((k1*(2*cc2 - 2*vu))/fc2^2 + (2*k2*((cc1 - uv)^2/fc1^2 + (cc2 - vu)^2/fc2^2)*(2*cc2 - 2*vu))/fc2^2)*(cc1 - uv)];
 
             
 
       
       dhvd_uuvu = [ ((k1*(2*cc1 - 2*uv))/fc1^2 + (2*k2*((cc1 - uv)^2/fc1^2 + (cc2 - vu)^2/fc2^2)*(2*cc1 - 2*uv))/fc1^2)*(cc2 - vu), ((k1*(2*cc2 - 2*vu))/fc2^2 + (2*k2*((cc1 - uv)^2/fc1^2 + (cc2 - vu)^2/fc2^2)*(2*cc2 - 2*vu))/fc2^2)*(cc2 - vu) + k1*((cc1 - uv)^2/fc1^2 + (cc2 - vu)^2/fc2^2) + k2*((cc1 - uv)^2/fc1^2 + (cc2 - vu)^2/fc2^2)^2 + 1];
 

dh_uuud = [[dhud_uuvu] ;
           [dhvd_uuvu] ];
       
 elseif method == 2
       
       %******************************************************************** 
       % Distortion Model used in:
       % A.J. Davison et.al. "Real-Time 3D SLAM with wide-angle vision"
       % EURON 2004
       
       % to be implemented!!
       
       
elseif method == 3
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         % SLAM Summer School 2006, Oxford.
         % Practical 3. SLAM using Monocular Vision.
         % Practical exercise.
         % J.M.M. Montiel, Javier Civera, Andrew J. Davison.
         % {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         
         
         % to be implemented!!
         
         
end;
