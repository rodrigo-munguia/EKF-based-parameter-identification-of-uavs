% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%

function y = barometer(uu,P)

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
    
    
   

    % simulate pressure sensors
    % pascal = 1 N/m^2 ,   hPa -> 100 Pa
    
  
   % 84050 N/m^2  -> 1602 m altitude at 25°c
   
   P_f = 84050; % N/m^2 barometric pressure at flight location
   T_c = 25; % Temperature at flight location celcius
   gravity = 9.81; % constant gravity
   %sigma_static_pres = 0.01e3; %100; % standard deviation of static pressure sensor in Kilo Pascals; MS5611-01BA03 datasheet
   
   %------------------------------------------
   B_abs_pres = 0; % temperature bias drift (N/m^2)
   %---------------------------------------------
   
   M = 0.0289644; % kg/mol standard molar mass of atmospheric air
   R = 8.31432; % N-m/mol-K universal gas constant for air
   
   T = T_c + 273.15; % Temperature kelvin
   
   rho = (M*P_f)/(R*T); % air density at flight location
   
  % h_AGL = -pd; %altitude above ground level
   h_AGL = -Z;
   
   
   a = .65/100 ; % K/m
   
   p_h1 = P_f*(1- (a*h_AGL)/T)^((M*gravity)/(R*a)); % eq 3 in Precise height measurement (24 bit) with pressure sensor MS5607 
    
   %y_static_pres = p_h1 + B_abs_pres + P.sigma_static_pres*randn();
   
   y_static_pres =   B_abs_pres + normrnd(p_h1,P.sigma_static_pres);
   
   %y_static_pres = rho*P.gravity*h_AGL + B_abs_pres ; %+ P.sigma_static_pres*randn(); % eq. 7.9
    
 
    

    % construct total output
    y = [...        
        y_static_pres;...        
    ];

end
