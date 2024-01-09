function drawAircraft_v2(uu)

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

    
     xmin = -.5;
     xmax = .5;  
     ymin = -.5; 
     ymax = .5;
     zmin = -.5;
     zmax = .5;
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1);
        hold on;
        cla
        
        
        
        axis([ xmin xmax ymin ymax zmin zmax]);
        set(gca,'YDir','reverse')   % change to NED coordintaes
        set(gca,'ZDir','reverse') 
        
        
        %Ryaw = Euler_to_Ra2b (0,0,-pi/4);
        
        Rw2r = Euler_to_Rw2r (phi,theta,psi);        
        
       %  Plot3DQuadRotor(.2,Rr2n,[0 0 0],'k',[.05 0 0]');       
         Plot3DQuadRotor(.2,Rw2r',[pn pe pd],'k',[.05 0 0]');    
         
        title('QUAD')
        xlabel('North')
        ylabel('East')
        zlabel('-Down')
       
         drawnow;
        
    % at every other time step, redraw base and rod
    else 
        
       figure(1);
       
       cla
       hold on;
        xmin = pn-.5;
        xmax = pn+.5;
        ymin = pe-.5;
        ymax = pe+.5;
        zmin = pd-.5;
        zmax = pd+.5;
       
        axis([ xmin xmax ymin ymax zmin zmax]);
        set(gca,'YDir','reverse')   % change to NED coordintaes
        set(gca,'ZDir','reverse') 
        
       % Ryaw = Euler_to_Ra2b (0,0,-pi/4);
        
           Rw2r = Euler_to_Rw2r (phi,theta,psi);        
        
       %  Plot3DQuadRotor(.2,Rr2n,[0 0 0],'k',[.05 0 0]');       
         Plot3DQuadRotor(.2,Rw2r',[pn pe pd],'k',[.05 0 0]');     
                                           
        title('QUAD')
        xlabel('North')
        ylabel('East')
        zlabel('-Down')
        
         
        drawnow;
        
         %drawnow;
         %pause(0.05);              
       
    end
end

