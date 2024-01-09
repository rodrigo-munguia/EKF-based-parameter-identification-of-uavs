function [uu vu] = Undistort_a_Point(uvd,cam_parameters,method)

    if method == 1
       %**********************************************************************************
       % Undistortion method used in Bouguet’s Calibration Toolbox for MATLAB® (Bouguet 2010)        
       % http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html 
       % WITHOUT tangencial distortion
       
       
       
       % EXAMPLE OF PARAMETERS FOR UNIBRAIN WEBCAM WITH WIDE_ANGLE LENS
       % cam_parameters.fc = [ 195.402084548286240 ; 195.368956684767080 ];
       % cam_parameters.cc = [ 159.973276434466810 ; 125.422686040349480 ]; % [u v]
       % cam_parameters.alpha_c = 0;
       % cam_parameters.distortions = [ -0.314522879219205 ; 0.087548258486875] ;
       %*****************************************************************************************
        kc = cam_parameters.distortions;
        cc = cam_parameters.cc;
        fc = cam_parameters.fc;
        
        ud = uvd(1);
        vd = uvd(2);
        
        
        % First: Subtract principal point, and divide by the focal length:
        um = (ud-cc(1))/fc(1);
        vm = (vd-cc(2))/fc(2);
        
        % Second: undo skew
        % x_distort(1,:) = x_distort(1,:) - alpha_c * x_distort(2,:);
        
        if length(kc) == 1, % for k1
            k2 = kc(1);
            radius_2= um.^2 + vm.^2;
            radial_distortion = 1 + ones(2,1)*(k2 * radius_2);
            radius_2_comp = (um.^2 + vm.^2) ./ radial_distortion(1,:);
            radial_distortion = 1 + ones(1,1)*(k2 * radius_2_comp);
           % x_comp = [um vd]./ radial_distortion;

            uum = um ./ radial_distortion;
            vum =  vm ./ radial_distortion;
        else % %for k1 and k2
            
            k1 = kc(1);
            k2 = kc(2);
            x = [um vm]; 				% initial guess
    
            for kk=1:20,
                
               %  r_2 = sum(x.^2);
               % k_radial =  1 + k1 * r_2 + k2 * r_2.^2 + k3 * r_2.^3;
               % delta_x = [2*p1*x(1,:).*x(2,:) + p2*(r_2 + 2*x(1,:).^2);
               % p1 * (r_2 + 2*x(2,:).^2)+2*p2*x(1,:).*x(2,:)];
               % x = (xd - delta_x)./(ones(2,1)*k_radial);
                
                r_2 = x(1)^2 + x(2)^2 ;
                k_radial =  1 + k1 * r_2 + k2 * r_2.^2 ;
                
                
                x(1) = um/k_radial;
                x(2) = vm/k_radial;

            end;
            uum = x(1);
            vum =  x(2);
            
            
        end
        
        uu = (fc(1)*uum + cc(1));
        vu = (fc(2)*vum  + cc(2));
        
        %uu  =round(uu);
        %vu = round(vu);
    
        
    
    elseif method == 2
       
       %******************************************************************** 
       % Undistortion Model used in:
       % A.J. Davison et.al. "Real-Time 3D SLAM with wide-angle vision"
       % EURON 2004
       %
       % EXAMPLE OF PARAMETERS FOR UNIBRAIN WEBCAM WITH WIDE_ANGLE LENS 
       % cam_parameters.fc = [ 195.402084548286240 ; 195.368956684767080 ];
       % cam_parameters.cc = [ 159.973276434466810 ; 125.422686040349480 ]; % [u v]
       % cam_parameters.alpha_c = 0;
       % cam_parameters.distortions = [6e-6 0];
       %********************************************************************
        kc = cam_parameters.distortions;
        cc = cam_parameters.cc;
        fc = cam_parameters.fc;
        
        ud = uvd(1);
        vd = uvd(2);
        
    
        r = sqrt((ud - cc(1))^2 + (vd - cc(2))^2 );

        uu = ((ud - cc(1))/sqrt(1 - 2*kc(1)*r^2)) + cc(1);
        vu = ((vd - cc(2))/sqrt(1 - 2*kc(1)*r^2)) + cc(2);
        
        % uu = round(uu);
        %vu = round(vu);
    
    elseif method == 3
         % Undistortion method used in:
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         % SLAM Summer School 2006, Oxford.
         % Practical 3. SLAM using Monocular Vision.
         % Practical exercise.
         % J.M.M. Montiel, Javier Civera, Andrew J. Davison.
         % {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % EXAMPLE OF PARAMETERS FOR UNIBRAIN WEBCAM WITH WIDE_ANGLE LENS 
        % cam_parameters.cc = [ 159.973276434466810 ; 125.422686040349480 ]; % [u v]
        % cam_parameters.fc = [ 2.2014 ; 2.2014];
        % cam_parameters.distortions = [6.931e-2 1.092e-2];
        % cam_parameters.pixel_size = 0.0112;
        
         
         kc = cam_parameters.distortions;
         cc = cam_parameters.cc;
         fc = cam_parameters.fc;
         d = cam_parameters.pixel_size;
         Cx = cc(1);
         Cy = cc(2);
         k1 = kc(1);
         k2 = kc(2);
        dx = cam_parameters.pixel_size;
        dy = cam_parameters.pixel_size;

        ud = uvd(1);
        vd = uvd(2);
        rd = sqrt( ( dx*(ud-Cx) )^2 + (dy*(vd-Cy) )^2 );

        uu = Cx + ( ud - Cx )*( 1 + k1*rd^2 + k2*rd^4 );
        vu = Cy + ( vd - Cy )*( 1 + k1*rd^2 + k2*rd^4 );

       
        
    end