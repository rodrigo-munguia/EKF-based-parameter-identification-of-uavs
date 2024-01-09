
% calculate JAcobians
clear all

syms pn pe pd u v w phi theta psi p q r real
syms w_1 w_2 w_3 w_4 w_5 w_6 real
syms Jx Jy Jz  b k real
syms g d mass Jxz real


% system state
x = [pn pe pd u v w phi theta psi p q r Jx Jy Jz b k]';

   

%---- moments and forces ------------------------------------------------

% gravity force expressed in the body frame 
    fg_b = [[ -mass*g*sin(theta) ]
            [  mass*g*cos(theta)*sin(phi)  ]
            [  mass*g*cos(theta)*cos(phi)  ]];
   
        
    % Quad - X    
    A = [[  b   b   b    b]
         [-d*b d*b d*b -d*b]
         [d*b -d*b d*b -d*b]
         [ k    k   -k    -k ]];
    
    % Quad + 
    %{
    A = [[  b   b   b    b]
         [ 0 -d*b 0  d*b]
         [d*b 0 -d*b  0]
         [ k    -k   k  -k ]]; 
     %}
    % Hexa x 
    %{  
     A = [[  b   b   b    b   b   b]
         [-d*b d*b d*b -d*b -d*b d*b]
         [  0  0   d*b -d*b  d*b -d*b ]
         [ -k  k  -k    k      k  -k ]];  
    %} 
     
     
   Tt = A*[w_1^2 w_2^2 w_3^2 w_4^2]';
  %  Tt = A*[w_1^2 w_2^2 w_3^2 w_4^2 w_5^2 w_6^2]';
 % propultion force
    
    % Eq. (4.18)
    
    f_b =  fg_b  - [0 0 Tt(1)]';
    
    fx = f_b(1); 
    fy = f_b(2); 
    fz = f_b(3);        
    
    ell = Tt(2); 
    m = Tt(3); 
    n = Tt(4); 


   %--------------------------
    
    T = Jx*Jz - Jxz^2;
    T1 = (Jxz*(Jx - Jy + Jz))/T;
    T2 = ((Jz*(Jz - Jy) + Jxz^2))/T;
    T3 = Jz/T;
    T4 = Jxz/T;
    T5 = (Jz - Jx)/Jy;
    T6 = Jxz/Jy;
    
    T7 = ((Jx-Jy)*Jx + Jxz^2)/T;
    T8 = Jx/T;
    
    %--------------------------
    
    % body to wold rotation matrix   (check)
    R_b2v = [[cos(theta)*cos(psi)  sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)  cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)];
             [cos(theta)*sin(psi)  sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)];
             [-sin(theta)    sin(phi)*cos(theta)   cos(phi)*cos(theta) ]];
    
    % 3.14
    pdot = R_b2v*[u v w]';
    
    pndot = pdot(1);
    
    pedot = pdot(2);
 
    pddot = pdot(3);
    
    %--------
    % 3.15
    udot = (r*v - q*w) +  (1/mass)*fx;
    
    vdot = (p*w - r*u) +  (1/mass)*fy; 
    
    wdot = (q*u - p*v) +  (1/mass)*fz;
   
    %--------
    % body rotational velocities to euler velocities Rotation matrix
    
    R_b2e = [[1   sin(phi)*tan(theta)   cos(phi)*tan(theta) ];
             [0        cos(phi)           -sin(phi)         ];
             [0   sin(phi)/cos(theta)   cos(phi)/cos(theta) ]];
    
    %R_b2e  = eye(3);     
         
    %3.16
    eudot =  R_b2e*[p q r]';    
    
    phidot = eudot(1);
    
    thetadot = eudot(2);
    
    psidot = eudot(3);
    
    %-----------------------------
    % 3.17
    
    pdot = (T1*p*q - T2*q*r) +  (T3*ell + T4*n);
    
    qdot = (T5*p*r - T6*(p^2 - r^2))  +  (1/Jy)*m;
    
    rdot = (T7*p*q  -  T1*q*r  )   +   (T4*ell + T8*n);
    
  
    %------------------------------------------------------------------------
    % System Dynamics
    
    % parameters dynamics
    Jxdot = 0;
    Jydot = 0;
    Jzdot = 0; 
    %Jxzdot = 0; 
    bdot = 0; 
    kdot = 0;
    
    
    
    x_dot = [ pndot pedot pddot udot vdot wdot phidot thetadot psidot pdot qdot rdot Jxdot Jydot Jzdot bdot kdot]';
    
    
    
    Jx =  jacobian( [x_dot], x)   
    
    
    Ju = jacobian( [x_dot], [w_1 w_2 w_3 w_4])   
    
    
    
    
    
    