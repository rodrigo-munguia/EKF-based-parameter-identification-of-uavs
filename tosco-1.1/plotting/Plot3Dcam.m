function Plot3Dcam(c_size,scale_ref,Rcam,Trans,colorcam)
%**************************************************************************  
% Rodrigo Munguia 2012
% Plots a 3D camera  
% size -> size of camera (metrics units)
% size_ref -> size of the axis of the coordinate reference frame 
% Rcam -> Camera to navigation rotation matrix
% Trans -> vector pointing from navigation origin torward camera origin
% colorcam -> color of the camera
%**************************************************************************
   a = c_size;
   
    
   % define camera
    x = [0 0 0 0 ; a/2 a/2  a/2 -a/2  ; -a/2 -a/2  a/2 -a/2  ; 0 0 0 0 ];
    y = [0 0 0 0 ;-a/2 a/2 -a/2 -a/2   ; -a/2  a/2 a/2 a/2     ;0 0 0 0 ];
    z = [0 0 0 0 ; a   a     a   a    ; a     a   a    a       ;0 0 0 0 ];

    xx = reshape(x,1,16);
    yy = reshape(y,1,16);
    zz = reshape(z,1,16);
    
    
    
    %rotate camera
    xyz = Rcam*[xx;yy;zz];
 
        
    %translate camera     
    cc = Trans;
    
    xyz(1,:) = xyz(1,:) + cc(1);
    xyz(2,:) = xyz(2,:) + cc(2);
    xyz(3,:) = xyz(3,:) + cc(3);
    
    %draw reference frame
    es = scale_ref;
    ax = [es*a  0 0]';
    axr = Rcam*ax + cc;
    line([cc(1) axr(1)],[cc(2) axr(2)],[cc(3) axr(3)],'linewidth',1,'color','g'); % green -> x 
    ay = [0  es*a 0]';
    ayr = Rcam*ay + cc;
    line([cc(1) ayr(1)],[cc(2) ayr(2)],[cc(3) ayr(3)],'linewidth',1,'color','b'); % blue -> y 
    az = [0  0 es*a]';
    azr = Rcam*az + cc;
    line([cc(1) azr(1)],[cc(2) azr(2)],[cc(3) azr(3)],'linewidth',1,'color','r'); % red -> z 
    
    %draw camera
    plot3(xyz(1,:),xyz(2,:),xyz(3,:),'LineWidth',2,'color',colorcam);
    
    
    
    