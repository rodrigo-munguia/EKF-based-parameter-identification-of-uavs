% Observability analysis

clear all

syms pn pe pd u v w phi theta psi p q r real
syms w_1 w_2 w_3 w_4 real
syms Jx Jy Jz Jxz b k real
syms g d mass real

% system state
x = [pn pe pd u v w phi theta psi p q r Jx Jy Jz Jxz b k]';

   

%---- moments and forces ------------------------------------------------

% gravity force expressed in the body frame 
    fg_b = [[ -mass*g*sin(theta) ]
            [  mass*g*cos(theta)*sin(phi)  ]
            [  mass*g*cos(theta)*cos(phi)  ]];
   
    A = [[  b   b   b    b]
         [-d*b d*b d*b -d*b]
         [d*b -d*b d*b -d*b]
         [ k    k   -k    -k ]];
     
   Tt = A*[w_1^2 w_2^2 w_3^2 w_4^2]';
   
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
    Jxzdot = 0; 
    bdot = 0; 
    kdot = 0;
    
    x_dot = [ pndot pedot pddot udot vdot wdot phidot thetadot psidot pdot qdot rdot Jxdot Jydot Jzdot Jxzdot bdot kdot]';
    
    
    % Output equations
    
    y1 = pn; 
    y2 = pe; 
    y3 = pd; 
    y4 = u;
    y5 = v; 
    y6 = w;
    y7 = phi;
    y8 = theta;
    y9 = psi;
    y10 =p; 
    y11 =q; 
    y12 =r;
    
    
    % Lie derivatives
    
    % eq 1
    dy1_dx = jacobian( [y1], x)    % lie 0
    y1dot = dy1_dx*x_dot;
    dy1dot_dx = jacobian( [y1dot], x) % lie 1
    y1dotdot = dy1dot_dx*x_dot;
    dy1dotdot_dx = jacobian( [y1dotdot], x) % lie 2
    
     % eq 2
    dy2_dx = jacobian( [y2], x)    % lie 0
    y2dot = dy2_dx*x_dot;
    dy2dot_dx = jacobian( [y2dot], x) % lie 1
    y2dotdot = dy2dot_dx*x_dot;
    dy2dotdot_dx = jacobian( [y2dotdot], x) % lie 2
        
     % eq 3
    dy3_dx = jacobian( [y3], x)    % lie 0
    y3dot = dy3_dx*x_dot;
    dy3dot_dx = jacobian( [y3dot], x) % lie 1
    y3dotdot = dy3dot_dx*x_dot;
    dy3dotdot_dx = jacobian( [y3dotdot], x) % lie 2
        
    % eq 4
    dy4_dx = jacobian( [y4], x)    % lie 0
    y4dot = dy4_dx*x_dot;
    dy4dot_dx = jacobian( [y4dot], x) % lie 1
    y4dotdot = dy4dot_dx*x_dot;
    dy4dotdot_dx = jacobian( [y4dotdot], x) % lie 2
    
    % eq 5
    dy5_dx = jacobian( [y5], x)    % lie 0
    y5dot = dy5_dx*x_dot;
    dy5dot_dx = jacobian( [y5dot], x) % lie 1
    y5dotdot = dy5dot_dx*x_dot;
    dy5dotdot_dx = jacobian( [y5dotdot], x) % lie 2
    
     % eq 6
    dy6_dx = jacobian( [y6], x)    % lie 0
    y6dot = dy6_dx*x_dot;
    dy6dot_dx = jacobian( [y6dot], x) % lie 1
    y6dotdot = dy6dot_dx*x_dot;
    dy6dotdot_dx = jacobian( [y6dotdot], x) % lie 2
    
      % eq 7
    dy7_dx = jacobian( [y7], x)    % lie 0
    y7dot = dy7_dx*x_dot;
    dy7dot_dx = jacobian( [y7dot], x) % lie 1
    y7dotdot = dy7dot_dx*x_dot;
    dy7dotdot_dx = jacobian( [y7dotdot], x) % lie 2
    
    
      % eq 8
    dy8_dx = jacobian( [y8], x)    % lie 0
    y8dot = dy8_dx*x_dot;
    dy8dot_dx = jacobian( [y8dot], x) % lie 1
    y8dotdot = dy8dot_dx*x_dot;
    dy8dotdot_dx = jacobian( [y8dotdot], x) % lie 2
    
    % eq 9
    dy9_dx = jacobian( [y9], x)    % lie 0
    y9dot = dy9_dx*x_dot;
    dy9dot_dx = jacobian( [y9dot], x) % lie 1
    y9dotdot = dy9dot_dx*x_dot;
    dy9dotdot_dx = jacobian( [y9dotdot], x) % lie 2
    
     % eq 10
    dy10_dx = jacobian( [y10], x)    % lie 0
    y10dot = dy10_dx*x_dot;
    dy10dot_dx = jacobian( [y10dot], x) % lie 1
    y10dotdot = dy10dot_dx*x_dot;
    dy10dotdot_dx = jacobian( [y10dotdot], x) % lie 2
    
     % eq 11
    dy11_dx = jacobian( [y11], x)    % lie 0
    y11dot = dy11_dx*x_dot;
    dy11dot_dx = jacobian( [y11dot], x) % lie 1
    y11dotdot = dy11dot_dx*x_dot;
    dy11dotdot_dx = jacobian( [y11dotdot], x) % lie 2
    
      % eq 12
    dy12_dx = jacobian( [y12], x)    % lie 0
    y12dot = dy12_dx*x_dot;
    dy12dot_dx = jacobian( [y12dot], x) % lie 1
    y12dotdot = dy12dot_dx*x_dot;
    dy12dotdot_dx = jacobian( [y12dotdot], x) % lie 2
    
    
    
    % Initial values
    
    pn0 = 1; 
    pe0 = 2; 
    pd0 = 1.5; 
    u0 = 1;
    v0 = 1.1; 
    w0 = .9;
    phi0 = .05;
    theta0 = .02;
    psi0 = .1;
    p0 = .1; 
    q0 = .11; 
    r0 = .09;
    Jx0 = .0001; 
    Jy0 = .0002; 
    Jz0 = .0003; 
    Jxz0 = .0004; 
    b0 = .01; 
    k0 = .02;
    
    x0 = [pn0 pe0 pd0 u0 v0 w0 phi0 theta0 psi0 p0 q0 r0 Jx0 Jy0 Jz0 Jxz0 b0 k0]';
    
    % known parameters
    g = 9.81; 
    d = 2;
    mass = 1;
    % inputs
    w_1 = 1;
    w_2 = 1.1;
    w_3 = 1.2;
    w_4 = 1.3;
    
    % numerical eval rows
    
    y1_0 = eval(subs(dy1_dx, x , x0 ))
    y1_1 = eval(subs(dy1dot_dx, x, x0 ))
    y1_2 = eval(subs(dy1dotdot_dx, x, x0 ))
    
    y2_0 = eval(subs(dy2_dx, x , x0 ))
    y2_1 = eval(subs(dy2dot_dx, x, x0 ))
    y2_2 = eval(subs(dy2dotdot_dx, x, x0 ))
    
    y3_0 = eval(subs(dy3_dx, x , x0 ))
    y3_1 = eval(subs(dy3dot_dx, x, x0 ))
    y3_2 = eval(subs(dy3dotdot_dx, x, x0 ))
    
    y4_0 = eval(subs(dy4_dx, x , x0 ))
    y4_1 = eval(subs(dy4dot_dx, x, x0 ))
    y4_2 = eval(subs(dy4dotdot_dx, x, x0 ))
    
    y5_0 = eval(subs(dy5_dx, x , x0 ))
    y5_1 = eval(subs(dy5dot_dx, x, x0 ))
    y5_2 = eval(subs(dy5dotdot_dx, x, x0 ))
    
    y6_0 = eval(subs(dy6_dx, x , x0 ))
    y6_1 = eval(subs(dy6dot_dx, x, x0 ))
    y6_2 = eval(subs(dy6dotdot_dx, x, x0 ))
    
    y7_0 = eval(subs(dy7_dx, x , x0 ))
    y7_1 = eval(subs(dy7dot_dx, x, x0 ))
    y7_2 = eval(subs(dy7dotdot_dx, x, x0 ))
    
    y8_0 = eval(subs(dy8_dx, x , x0 ))
    y8_1 = eval(subs(dy8dot_dx, x, x0 ))
    y8_2 = eval(subs(dy8dotdot_dx, x, x0 ))
    
    y9_0 = eval(subs(dy9_dx, x , x0 ))
    y9_1 = eval(subs(dy9dot_dx, x, x0 ))
    y9_2 = eval(subs(dy9dotdot_dx, x, x0 ))
    
    y10_0 = eval(subs(dy10_dx, x , x0 ))
    y10_1 = eval(subs(dy10dot_dx, x, x0 ))
    y10_2 = eval(subs(dy10dotdot_dx, x, x0 ))
       
    y11_0 = eval(subs(dy11_dx, x , x0 ))
    y11_1 = eval(subs(dy11dot_dx, x, x0 ))
    y11_2 = eval(subs(dy11dotdot_dx, x, x0 ))
    
    y12_0 = eval(subs(dy12_dx, x , x0 ))
    y12_1 = eval(subs(dy12dot_dx, x, x0 ))
    y12_2 = eval(subs(dy12dotdot_dx, x, x0 ))
    
    
    %Q = [y1_0; y2_0; y3_0 ; y4_0 ; y5_0 ; y6_0 ; y7_0 ; y8_0 ; y9_0; y10_0 ; y11_0 ; y12_0] 
    Q = [y1_0; y2_0; y3_0 ; y4_0 ; y5_0 ; y6_0 ; y6_1; y7_0 ; y8_0 ; y9_0; y10_0 ; y10_1 ; y10_2; y11_0 ; y11_1; y11_2;  y12_0; y12_1 ; y12_2 ] 
    
    
    rank(Q)
    