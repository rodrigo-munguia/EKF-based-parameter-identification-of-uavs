function [ud vd] = Distort_a_Point(uv,cam_parameters,method)

    if method == 1
       %**********************************************************************************
       % Distortion model used in Bouguet’s Calibration Toolbox for MATLAB® (Bouguet 2010)        
       % http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html 
       % WITHOUT tangencial distortion
       
       %The pinehole projection coordinates of P is [a;b] where a=x/z and b=y/z.
        %call r^2 = a^2 + b^2.
        %The distorted point coordinates are: xd = [xx;yy] where:
        %
        %xx = a * (1 + kc(1)*r^2 + kc(2)*r^4)      +      2*kc(3)*a*b + kc(4)*(r^2 + 2*a^2);
        %yy = b * (1 + kc(1)*r^2 + kc(2)*r^4)      +      kc(3)*(r^2 + 2*b^2) + 2*kc(4)*a*b;
        %
        %The left terms correspond to radial distortion, the right terms correspond to tangential distortion
        %
        %Fianlly, convertion into pixel coordinates: The final pixel coordinates vector xp=[xxp;yyp] where:
        %
        %xxp = f(1)*xx + c(1)
        %yyp = f(2)*yy + c(2)
       
       % EXAMPLE OF PARAMETERS FOR UNIBRAIN WEBCAM WITH WIDE_ANGLE LENS
       % cam_parameters.fc = [ 195.402084548286240 ; 195.368956684767080 ];
       % cam_parameters.cc = [ 159.973276434466810 ; 125.422686040349480 ]; % [u v]
       % cam_parameters.alpha_c = 0;
       % cam_parameters.distortions = [ -0.314522879219205 ; 0.087548258486875] ;
       %*****************************************************************************************
        kc = cam_parameters.distortions;
        cc = cam_parameters.cc;
        fc = cam_parameters.fc;
        
        u = uv(1);
        v = uv(2);
        
        um = (u-cc(1))/fc(1);
        vm = (v-cc(2))/fc(2);

        r = sqrt(um*um + vm*vm);

        umd = um * (1 + kc(1)*r^2 + kc(2)*r^4);
        vmd = vm * (1 + kc(1)*r^2 + kc(2)*r^4);

        ud = fc(1)*umd + cc(1);
        vd = fc(2)*vmd + cc(2);
    
    elseif method == 2
       
       %******************************************************************** 
       % Distortion Model used in:
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
        
        u = uv(1);
        v = uv(2);
        
    
        r = sqrt((u - cc(1))^2 + (v - cc(2))^2 );

        ud = ((u - cc(1))/sqrt(1 + 2*kc(1)*r^2)) + cc(1);
        vd = ((v - cc(2))/sqrt(1 + 2*kc(1)*r^2)) + cc(2);
        
        
    
    elseif method == 3
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
        
         uvu = uv;
         kc = cam_parameters.distortions;
         cc = cam_parameters.cc;
         fc = cam_parameters.fc;
         d = cam_parameters.pixel_size;
         Cx = cc(1);
         Cy = cc(2);
         k1 = kc(1);
         k2 = kc(2);
         
          dx = d;
          dy = d;

          xu=(uvu(1)-Cx)*dx;
          yu=(uvu(2)-Cy)*dy;

          ru=sqrt(xu*xu+yu*yu);
          rd=ru/(1+k1*ru^2+k2*ru^4);
          for k=1:100
              f=rd+k1*rd^3+k2*rd^5-ru;
              f_p=1+3*k1*rd^2+5*k2*rd^4;
              rd=rd -f/f_p;
          end

          D=1+k1*rd^2+k2*rd^4;
          xd=xu/D;
          yd=yu/D;

          ud = xd/dx+Cx; 
          vd = yd/dy+Cy;

        
    end