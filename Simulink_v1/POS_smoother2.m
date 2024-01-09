% estimate_states
%   - GPS smothing
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 

% 
% 
%

function xhat = POS_smoother2(uu,P)

   % rename inputs
   y_gps_n       = uu(1);
   y_gps_e       = uu(2);
   y_gps_h       = uu(3);
   
   y_bar_h       = uu(4);   
   
   y_phi       = uu(5);
   y_theta       = uu(6);
   y_psi       = uu(7);  
   
   ax_b = uu(8);
   ay_b = uu(9);
   az_b = uu(10);
   
   t             = uu(11);  
  
   
    
    %**********************************************************************
    % GPS Smoothing
    % x_hat_g = [pn pe pd dpn dpe dpd]  
    
    persistent x_hat_g P_g last_gps_r last_bar_a;    
    %Initialization
    
    if t == 0  
        x_hat_g = [y_gps_n  y_gps_e y_gps_h  0  0   0 ]';
       % x_hat_g = [0  0 0  0  0   0 ]';
        last_gps_r = [y_gps_n  y_gps_e y_gps_h  ]';
        last_bar_a = y_bar_h;
        P_g = diag([eps eps eps eps eps eps ]);
    end  
     % EKF Prediction Step
    N = 1; % integrate prediction equation N times each P.Ts seconds
    Q = diag([.05^2 .05^2 .05^2 .2^2 .2^2 .2^2]');   % Covariance matrix for zero-mean Gaussian random process
    
    
   
    
    
    pn = x_hat_g(1);
    pe = x_hat_g(2);
    pd = x_hat_g(3);
    u = x_hat_g(4);
    v = x_hat_g(5);
    w =  x_hat_g(6);
    % R = diag([ P.sigma_n_gps^2 P.sigma_e_gps^2 P.sigma_Vg_gps^2 (P.sigma_Vg_gps/Vg)^2 (5)^2  (5)^2 ]);
    
    dt = P.GPSsm.dt;
    g = P.gravity;
    
    
    for i=1:N 
      
        
      Rw2b = Euler_to_Rw2r (y_phi,y_theta,y_psi);
      Rbrw = Rw2b';  
        
     f_x_u(1:3) = Rbrw*[u v w]';
     %f_x_u(4:6) = [ax_b ay_b az_b]' + Rw2b*[0 0 g]';
     %f_x_u(4:6) = [ax_b ay_b az_b]' ;
      f_x_u(4:6) = [0 0 0]' ;
      
      Fx= zeros(6);
      Fx(1:3,4:6) = Rbrw;
      
      
      %f_x_u = A*[pn pe pd dpn dpe dpd]';
       
      x_hat_g = x_hat_g + (P.GPSsm.dt/N)*f_x_u';    
      
      P_g = P_g + (P.GPSsm.dt/N)*(Fx*P_g + P_g*Fx' + Q);       
          
    end
    
   % EKF Measurement Step
   
   y_gps = [y_gps_n  y_gps_e y_gps_h  ]';
   
   
   if ~isequal(y_gps,last_gps_r) % we got new gps data
    
       R =  diag([(P.sigma_n_gps)^2 P.sigma_e_gps^2 1*P.sigma_h_gps^2]'); 
       
       H = [[1 0 0 0 0 0]
           [0 1 0 0 0 0]
           [0 0 1 0 0 0]];
    
   
       
       h = [[x_hat_g(1)]
            [x_hat_g(2)]
            [x_hat_g(3)]];
       
       L = P_g*H'/((R + H*P_g*H'));
        
       P_g = (eye(6) - L*H)*P_g;
       
       x_hat_g = x_hat_g + L*(y_gps - h); 
     
       
       last_gps_r = y_gps;
   end
   %}
   
    if ~isequal(y_bar_h ,last_bar_a) % we got new barometer data
        
        R = (P.sigma_static_pres*.1)^2;
        H = [0 0 1 0 0 0];
        h = x_hat_g(3);
        
        L = P_g*H'/((R + H*P_g*H'));
        
       P_g = (eye(6) - L*H)*P_g;
       
       x_hat_g = x_hat_g + L*(y_bar_h - h); 
       disp([y_bar_h  h]) 
       
        last_bar_a = y_bar_h; 
    end    
  
   %-----
   pn = x_hat_g(1);
    pe = x_hat_g(2);
    pd = x_hat_g(3);
    u = x_hat_g(4);
    v = x_hat_g(5);
    w =  x_hat_g(6);
        
   %***********************************************************************  
    %}
    
     xhat = [pn pe pd u v w];
end