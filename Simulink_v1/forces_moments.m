% forces_moments.m
%   Computes the foreces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%
%  Revised:
%   2/2/2010 - RB 
%   5/14/2010 - RB

function out = forces_moments(x, motor, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    m_1 = motor(1);
    m_2 = motor(2);
    m_3 = motor(3);
    m_4 = motor(4);
    
    
    % No wind taken into account !!
    
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    
    % robot to wold rotation matrix 
    R_r2w = [[cos(theta)*cos(psi)  sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)  cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)];
             [cos(theta)*sin(psi)  sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)];
             [-sin(theta)    sin(phi)*cos(theta)   cos(phi)*cos(theta) ]];
    R_w2r = R_r2w'; 
    
    
    % wind vector expresed in the body frame
    Vw_b = R_w2r*[w_ns w_es w_ds]' + [u_wg v_wg w_wg ]'; 
    
    % compute wind vector in the inertial frame
    Vw_i = R_r2w*Vw_b;
    
    w_n = Vw_i(1);
    w_e = Vw_i(2);
    w_d = Vw_i(3);
    
    
   
    %--------------------------------------------
    
    g = P.gravity;
    m = P.mass;
    
    % gravity force expressed in the body frame 
    fg_b = [[ -m*g*sin(theta) ]
            [  m*g*cos(theta)*sin(phi)  ]
            [  m*g*cos(theta)*cos(phi)  ]];
        
        
   %Rw2r = Euler_to_Rw2r(phi,theta,psi);     
    
   %fg_b = Rw2r*[0 0 m*g]';
   
  
   
  
  d = P.d ; % m   Lenght arm
  b = P.b ;% N/rad/s Lift (thrust) factor
  k = P.k ; %K N.m/rad/s Drag factor
        
    
    A = [[  b   b   b    b]
         [-d*b d*b d*b -d*b]
         [d*b -d*b d*b -d*b]
         [ k    k   -k    -k ]];
     
   Tt = A*[m_1^2 m_2^2 m_3^2 m_4^2]';
   
   
    
    % propultion force
    
    
    % Eq. (4.18)
    
    f_b =  fg_b  - [0 0 Tt(1)]';
    
    %f_b
         
    %f_b(3) = -1;      
    Force = [f_b(1); f_b(2); f_b(3)];
    
      
    
    Torque = [Tt(2); Tt(3); Tt(4)];
    
    Va = 0;
    alpha = 0;
    beta = 0;
    
    
    out = [Force; Torque; Va; alpha; beta; w_n; w_e; w_d];

end



