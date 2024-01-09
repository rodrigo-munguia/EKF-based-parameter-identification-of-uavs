% estimate_states
%   - GPS smothing
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 

% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = POS_smoother(uu,P)

   % rename inputs
   y_gps_n       = uu(1);
   y_gps_e       = uu(2);
   y_gps_h       = uu(3);
   
   y_bar_h       = uu(4);
   
   t             = uu(5);  
  
   
    
    %**********************************************************************
    % GPS Smoothing
    % x_hat_g = [pn pe pd dpn dpe dpd]  
    
    persistent x_hat_g P_g last_gps_r last_bar_a;    
    %Initialization
    
    if t == 0  
        x_hat_g = [y_gps_n  y_gps_e y_gps_h  0  0   0 ]';
        last_gps_r = [y_gps_n  y_gps_e y_gps_h  ]';
        last_bar_a = y_bar_h;
        P_g = diag([eps eps eps eps eps eps ]);
    end  
     % EKF Prediction Step
    N = 2; % integrate prediction equation N times each P.Ts seconds
    Q = diag([.2^2 .1^2 .1^2 .2^2 .2^2 .2^2]');   % Covariance matrix for zero-mean Gaussian random process
    
    
   
    
    
    pn = x_hat_g(1);
    pe = x_hat_g(2);
    pd = x_hat_g(3);
    dpn = x_hat_g(4);
    dpe = x_hat_g(5);
    dpd =  x_hat_g(6);
    % R = diag([ P.sigma_n_gps^2 P.sigma_e_gps^2 P.sigma_Vg_gps^2 (P.sigma_Vg_gps/Vg)^2 (5)^2  (5)^2 ]);
    
    dt = P.GPSsm.dt;
    for i=1:N 
      
      A=   [[0 0 0 1 0 0]
           [0 0 0 0 1 0]
           [0 0 0 0 0 1]
           [0 0 0 0 0 0]
           [0 0 0 0 0 0]
           [0 0 0 0 0 0]];
      
      f_x_u = A*[pn pe pd dpn dpe dpd]';
       
      x_hat_g = x_hat_g + (P.GPSsm.dt)*f_x_u;    
      
      P_g = P_g + (P.GPSsm.dt/N)*(A*P_g + P_g*A' + Q);       
          
    end
    
   % EKF Measurement Step
   
   y_gps = [y_gps_n  y_gps_e y_gps_h  ]';
   
   if ~isequal(y_gps,last_gps_r) % we got new gps data
    
       R =  diag([(P.sigma_n_gps)^2 P.sigma_e_gps^2 100*P.sigma_h_gps^2]'); 
       
       H = [[1 0 0 0 0 0]
           [0 1 0 0 0 0]
           [0 0 1 0 0 0]];
    
   
       
       h = [[x_hat_g(1)]
            [x_hat_g(2)]
            [x_hat_g(3)]];
       
       L = P_g*H'/((R + H*P_g*H'));
        
       P_g = (eye(6) - L*H)*P_g;
       
       x_hat_g = x_hat_g + L*(y_gps - h); 
      %}
       
       last_gps_r = y_gps;
   end
   
    if ~isequal(y_bar_h ,last_bar_a) % we got new barometer data
        
        R = (P.sigma_static_pres*.1)^2;
        H = [0 0 1 0 0 0];
        h = x_hat_g(3);
        
        L = P_g*H'/((R + H*P_g*H'));
        
       P_g = (eye(6) - L*H)*P_g;
       
       x_hat_g = x_hat_g + L*(y_bar_h - h); 
        
        last_bar_a = y_bar_h; 
    end    
   
   
   %-----
   pn = x_hat_g(1);
    pe = x_hat_g(2);
    pd = x_hat_g(3);
    dpn = x_hat_g(4);
    dpe = x_hat_g(5);
    dpd =  x_hat_g(6);
        
   %***********************************************************************  
    %}
    
     xhat = [pn pe pd dpn dpe dpd];
end