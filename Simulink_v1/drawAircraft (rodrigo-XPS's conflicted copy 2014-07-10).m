function drawAircraft(uu)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

    % define persistent variables 
    persistent Aircraft_handle;
     persistent Fx_handle Fy_handle Fz_handle;
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        Aircraft_handle = drawAircraftBody(pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        [Fx_handle Fy_handle Fz_handle] = drawAircraftRefFrame(pn,pe,pd,phi,theta,psi,[],[],[]);                                   
                                           
        title('Aircraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
       % axis([-100,100,-100,100,-100,100]);
                
        %set(gca,'YDir','Reverse')
        hold on
        
    % at every other time step, redraw base and rod
    else 
        
        figure(1)
        sizea = 30;
        R = [...
            0, 1, 0;...
            1, 0, 0;...
            0, 0, -1;...
            ];
                                   
       pxyz = R*[pn pe pd]';
       px = pxyz(1);
       py = pxyz(2);
       pz = pxyz(3);
       
        
      
                      
       %axis([(pe -(sizea/2)),(pe + (sizea/2)),( pn -(sizea/2)), ( pn +(sizea/2)), (pd -(sizea/2)),(pd +(sizea/2))]);                              
        
        axis([(px -(sizea/2)),(px + (sizea/2)),( py -(sizea/2)), ( py +(sizea/2)), (pz -(sizea/2)),(pz +(sizea/2))]);     
        
        drawAircraftBody(pn,pe,pd,phi,theta,psi,...
                           Aircraft_handle);
        drawAircraftRefFrame(pn,pe,pd,phi,theta,psi,Fx_handle,Fy_handle ,Fz_handle);  
        
         
        drawnow
        
                       
       
    end
end

%----------------------------------------------------------------------
% Draw body reference frame
function [handleX handleY handleZ] = drawAircraftRefFrame(pn,pe,pd,phi,theta,psi,...
                                       handleX, handleY, handleZ)
     
          
     vcx = [0 0 0;
           3 0 0];
  
      vcx = rotate(vcx', phi, theta, psi)'; 
      vcx = translate(vcx', pn, pe, pd)';
      R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
      vcx = vcx*R;  
      
      vcy = [0 0 0;
           0 3 0];
  
      vcy = rotate(vcy', phi, theta, psi)'; 
      vcy = translate(vcy', pn, pe, pd)';
      R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
      vcy = vcy*R;    
      
      vcz = [0 0 0;
           0 0 3];
  
      vcz = rotate(vcz', phi, theta, psi)'; 
      vcz = translate(vcz', pn, pe, pd)';
      R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
      vcz = vcz*R;    
  
  
  if isempty(handleX),
    handleX = line(vcx(:,1),vcx(:,2),vcx(:,3),'linewidth',1,'color','g'); % green -> x
    handleY = line(vcy(:,1),vcy(:,2),vcy(:,3),'linewidth',1,'color','b'); % blue-> y
    handleZ = line(vcz(:,1),vcz(:,2),vcz(:,3),'linewidth',1,'color','r'); % red-> z
  else
    set(handleX,'XData',vcx(:,1),'YData',vcx(:,2),'ZData',vcx(:,3));
      set(handleY,'XData',vcy(:,1),'YData',vcy(:,2),'ZData',vcy(:,3));
        set(handleZ,'XData',vcz(:,1),'YData',vcz(:,2),'ZData',vcz(:,3));
    drawnow
  end
end
  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawAircraftBody(pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  [V,F, patchcolors] = AircraftVFC;
  
  V = rotate(V', phi, theta, psi)';  % rotate 
    V = translate(V', pn, pe, pd)';  % translate 
  

  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;
  
  
  if isempty(handle),
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
   % set(handle,'XData',vc(:,1),'YData',vc(:,2),'ZData',vc(:,3));
   % drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi);
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_yaw*R_pitch*R_roll;
  % rotate vertices
  XYZ = R*XYZ;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)
  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

  