% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%

function y = Pressure2Altitude(uu,P)

    y_static_pres   = uu(1);
    t =  uu(2);
    
    
   

 M = 0.0289644; % kg/mol standard molar mass of atmospheric air
 R = 8.31432; % N-m/mol-K universal gas constant for air
 a = .65/100 ; % K/m
 gravity = 9.80665;
 % 
  %---------------------
 h_ASL =  1602 ; % altitude over sea level (zapopan)
 T0 = 288.15; %  Kelvin standard temperature at sea level
 P0 = 101325; % N/m^2 standard pressure at sea level
 L0 = -0.0065; % K/m lapse rate of temperature deacrese in lower atmosphere
 
  % This parameters should be obtained from at the flight location
 P_f = 84050; % N/m^2 barometric pressure at flight location
 T_c = 25; % Temperature at flight location celcius  % Temperature at flight location celcius
 
 %rho = (M*P)/(R*T); % air density at flight location
 % P = P0*( T0/(T0 + L0*h_ASL ) )^ep ; % atmosphere pressure at flight location
 
 T = T_c + 273.15; % Temperature kelvin    

 % Equation 1 (in function of initial read pressure )
 
 ee = (R*L0)/(M*gravity);
 hhat = (1 -  (y_static_pres/P_f).^ee )*(T/a); % relative altitute
 
 %global initPos SampleTime
 persistent zf alfa_LPV_p ;
 
 
 if t == 0       
     a_p = 2; % frequency cut
     zf = P.pd0;  % initial Down position (negative altitude);   
     alfa_LPV_p = exp(-a_p*P.Ts_Barometer);         
end    
 

 
 zf = alfa_LPV_p *zf + (1-alfa_LPV_p )*hhat;
 
 
 q = 10;   

    % construct total output
    y = [...        
        zf;hhat;      
    ];

end
