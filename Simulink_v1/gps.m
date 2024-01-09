% gps.m
%   Compute the output of gps sensor
%

function y = gps(uu,P)

    % relabel the inputs
   X      = uu(1);
   Y      = uu(2);
   Z      = uu(3);
    dx       = uu(4);
    dy      = uu(5);
    dz       = uu(6);
    phi     = uu(7);
    theta   = uu(8);
    psi     = uu(9);
    p       = uu(10);
    q       = uu(11);
    r       = uu(12);
    
    %{
    X      = uu(1);
    Y      = uu(2);
    Z      =  uu(3);
    yaw = uu(4);
    pitch = uu(5);
    roll = uu(6);
    dx = uu(7);
    dy = uu(8);
    dz = uu(9);
    dyaw = uu(10);
    dpitch = uu(11);
    droll = uu(12);
     %}
    t       = uu(13);
    
    pn      = X;
    pe      = Y;
    pd      = Z;
    
    Va      = sqrt(dx^2 + dy^2 + dz^2);
    
    if (Va == 0)
     Va = .00001;
    end
        
    
    
    
    wn      = 0;
    we      = 0;
%    wd      = 0;
   

  


   % GPS parameters

   
   
    
    % stuff goes here
    
    persistent vn ve vh
    
    if t == 0
       vn = .001;
       ve = .001;
       vh = .002;        
    end
    
    
    vn = exp(-P.K_gps*P.Ts_gps)*vn + P.sigma_n_gps*randn();
    ve = exp(-P.K_gps*P.Ts_gps)*ve + P.sigma_e_gps*randn();
    vh = exp(-P.K_gps*P.Ts_gps)*vh + P.sigma_h_gps*randn();
    

    % construct North, East, and altitude GPS measurements
    y_gps_n = pn + vn;
    y_gps_e = pe + ve; 
    y_gps_h = pd + vh; 
   %  y_gps_n = pn ;
   % y_gps_e = pe ; 
   % y_gps_h = pd ; 
    
    % construct groundspeed and course measurements
    Vg = sqrt((Va*cos(psi) + wn)^2 + (Va*sin(psi) + we)^2 );
    
    y_gps_Vg     = Vg + P.sigma_Vg_gps*randn(); % eq. 725
    
    if Vg < 1
       Vg = 1; 
    end    
    
    y_gps_course = atan2(Va*sin(psi) + we , Va*cos(psi) + wn )  + (P.sigma_Vg_gps/Vg)*randn() ; % eq. 7.26

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



