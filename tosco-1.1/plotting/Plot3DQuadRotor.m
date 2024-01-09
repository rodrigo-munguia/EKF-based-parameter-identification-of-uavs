function Plot3DQuadRotor(f_size,Rr2n ,Tn2r,colorframe,xref)
%**************************************************************************  
% Rodrigo Munguia 2012
% Plots a 3D QuadRotor   
% r_size -> size of the arm of the quadcopter
% Rr2n -> Robot to navigation rotation matrix [3x3]
% Tn2r -> Robot to navigation translation vector  [x y z]'
% colorframe -> color of the quadrotor
% xref -> x reference for quad [x y z]',   vector defined in the vehicle
% frame, pointing toward the x axis
%**************************************************************************
   a = f_size;
   
   
   Ryaw = Euler_to_Ra2b (0,0,-pi/4); % X quad rotation
   
   % define camera
    
    x = [0  0; a  0 ;  -a  0 ];
    y = [0  0; 0  a ;  0  -a];
    z = [0  0; 0  0 ;  0  0];

    
    xx = reshape(x,1,6);
    yy = reshape(y,1,6);
    zz = reshape(z,1,6);
    
    %rotate camera
    xyz = Rr2n*Ryaw*[xx;yy;zz];
 
        
    %translate camera     
    cc = Tn2r;
    
    xyz(1,:) = xyz(1,:) + cc(1);
    xyz(2,:) = xyz(2,:) + cc(2);
    xyz(3,:) = xyz(3,:) + cc(3);
    
    plot3(xyz(1,:),xyz(2,:),xyz(3,:),'LineWidth',2,'color',colorframe);
    
    
 centers = [[a 0 -.05]
           [-a 0 -.05] 
           [0  a -.05]
           [0 -a -.05]];
 normal = [0 0 1];
 radius = a*.5;
 
 
 
 for i=1:4
    center = centers(i,:);
    theta=0:0.01:2*pi;
    v=null(normal);
    points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
    xyz = Rr2n*Ryaw*points;
    xyz(1,:) = xyz(1,:) + cc(1);
    xyz(2,:) = xyz(2,:) + cc(2);
    xyz(3,:) = xyz(3,:) + cc(3);
    plot3(xyz(1,:),xyz(2,:),xyz(3,:),'LineWidth',1,'color',colorframe,'LineStyle','-');
 end
  
  vc = [a 0 0]';
  xyz = Ryaw*vc;
  xyz = Rr2n*xyz + cc';
  % plot x direction (green)
  plot3( xyz(1,:),xyz(2,:),xyz(3,:), 'o', 'MarkerEdgeColor','k', 'MarkerFaceColor', 'g', 'MarkerSize', 4 );
  vc = [0 a 0]';
  xyz = Rr2n*Ryaw*vc;
  xyz = xyz + cc';
  % plot y direction (blue)
  plot3( xyz(1,:),xyz(2,:),xyz(3,:), 'o', 'MarkerEdgeColor','k', 'MarkerFaceColor', 'r', 'MarkerSize', 4 );
  vc = [-a 0 0]';
  xyz = Rr2n*Ryaw*vc;
  xyz = xyz + cc';
  plot3( xyz(1,:),xyz(2,:),xyz(3,:), 'o', 'MarkerEdgeColor','k', 'MarkerFaceColor', 'r', 'MarkerSize', 4 ); 
  vc = [0 -a 0]';
  xyz = Rr2n*Ryaw*vc;
  xyz = xyz + cc';
  plot3( xyz(1,:),xyz(2,:),xyz(3,:), 'o', 'MarkerEdgeColor','k', 'MarkerFaceColor', 'r', 'MarkerSize', 4 );
 
  
  
  % vehicle reference
  
  axr = Rr2n*xref + cc';
  line([cc(1) axr(1)],[cc(2) axr(2)],[cc(3) axr(3)],'linewidth',3,'color','g'); % green -> x
  
  Ryaw =  [[ 0.0000   -1.0000    0]
           [1.0000    0.0000     0]
           [0         0    1.0000 ]];
          
  
  yref = Ryaw*xref;
       
  ayr =    Rr2n*yref + cc';
  line([cc(1) ayr(1)],[cc(2) ayr(2)],[cc(3) ayr(3)],'linewidth',3,'color','b'); % blue -> y
  
  d = dot(axr,ayr);
  
 
   zref = cross(xref,yref);
   zref = (zref/norm(zref))*norm(xref);

  azr = Rr2n*zref + cc';
  
  line([cc(1) azr(1)],[cc(2) azr(2)],[cc(3) azr(3)],'linewidth',3,'color','r'); % red -> z

    
    