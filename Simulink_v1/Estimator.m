function Estimator(block)
%************************************
%  Implements a (DISCRETE) proportional-integral controler for attitude


% Level-2 MATLAB file S-Function for limited integrator demo.
%   Copyright 1990-2009 The MathWorks, Inc.
%   $Revision: 1.1.6.2 $ 

  setup(block);
  
%endfunction

function setup(block)
  %global PAR;
  %% Register number of dialog parameters   
  block.NumDialogPrms = 1;

  %% Register number of input and output ports
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = [4 1];
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = [15 1];
  block.InputPort(2).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = [17 1];
  
  %% Set block sample time to continuous
  %block.SampleTimes = [PAR.Calt_Ts  0];
  P = block.DialogPrm(1).Data;  % get parameters
  dt = P.est_dt;
  
  block.SampleTimes = [dt  0];
  
  %% Setup Dwork
  block.NumContStates = 0;  % the integral states 

  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('Start',                   @Start);
  block.RegBlockMethod('Outputs',                 @Output);  
  %block.RegBlockMethod('Derivatives',             @Derivative);  
  block.RegBlockMethod('Update',                  @Update);
  block.RegBlockMethod('PostPropagationSetup',  @PpropagationS);
  
%endfunction
function PpropagationS(block)

 % Setup Dwork
  block.NumDworks                = 3;
  
  block.Dwork(1).Name            = 'x'; 
  block.Dwork(1).Dimensions      = 17;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'P'; 
  block.Dwork(2).Dimensions      = 17*17;
  block.Dwork(2).DatatypeID      = 0;
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'flags'; 
  block.Dwork(3).Dimensions      = 5;
  block.Dwork(3).DatatypeID      = 0;
  block.Dwork(3).Complexity      = 'Real';
  block.Dwork(3).UsedAsDiscState = true;
  

function Start(block)

  %% Initialize Dwork

 %{
P.Jx   = 0.0224; %kg.m2
P.Jy   = 0.0224; %kg.m2
P.Jz   = 0.0436;  %kg.m2
P.Jxz  = 0;  %kg.m2
P.b = 3.2; % N/rad/s Lift (thrust) factor
P.k = .064; %K N.m/rad/s Drag factor
%}  
   block.Dwork(1).Data = [0 0 0 0 0 0 0 0 0 0 0 0 .005 .005 .005 1 0.01];  % initialize x ; initialize state
   
   P_ini = diag([1 1 1 .1 .1 .1 .1 .1 .1 .1 .1 .1 .001 .001 .001 .01 .01]);  %  initialize covarianze matrix
   
   
   block.Dwork(2).Data  = reshape(P_ini,17*17,1);
   
   
   y_pos = block.InputPort(2).Data(1:3);  % position
   y_vel = block.InputPort(2).Data(4:6);  % velocity
   y_a = block.InputPort(2).Data(6:9);  % acelerometers
   y_att = block.InputPort(2).Data(10:12); % measured euler angles
   y_av = block.InputPort(2).Data(13:15); % Angular velocyty (gyros)
   
   block.Dwork(3).Data(1) = mean(y_pos);
   block.Dwork(3).Data(2) = mean(y_a);
   block.Dwork(3).Data(3) = mean(y_att);
   block.Dwork(3).Data(4) = mean(y_av);
   block.Dwork(3).Data(5) = mean(y_vel);
   
  q = 10;
  
  
%block.ContStates.Data(1) = PAR.iniSt.thetar_r;

  
  
  
function Output(block)


P = block.DialogPrm(1).Data;  % get parameters
%x = block.InputPort(2).Data ; % state feedback

% north control
%{
xi_pn = block.Dwork(1).Data;  %get xi state

xa = [x(1);x(3);xi_pn]; % augmented state



k = P.C.K_pn;
ki = P.C.Ki_pn;

K1 = [k -ki];
%u = Ki*xi - K*xa;
u_pn = -K1*xa; 

disp(u_pn)
%-----------------
% east control

xi_pe = block.Dwork(2).Data;  %get xi state

xa = [x(2);x(4);xi_pe]; % augmented state

k = P.C.K_pe;
ki = P.C.Ki_pe;

K1 = [k -ki];
%u = Ki*xi - K*xa;
u_pe = -K1*xa; 

%------------------------------------

%}

block.OutputPort(1).Data = block.Dwork(1).Data;

  
%endfunction

function Update(block)

Par = block.DialogPrm(1).Data;  % get parameters
dt = Par.est_dt;


uu = block.InputPort(1).Data ; % control input

y_pos = block.InputPort(2).Data(1:3);  % position
y_vel = block.InputPort(2).Data(4:6);  % velocity
y_a = block.InputPort(2).Data(7:9);  % acelerometers
y_att = block.InputPort(2).Data(10:12); % measured euler angles
y_av = block.InputPort(2).Data(13:15); % Angular velocyty (gyros)

flags = block.Dwork(3).Data;

x = block.Dwork(1).Data; % system state

P = reshape(block.Dwork(2).Data,17,17); % covariance matrix



% check for new measurements
new_pos = 0;
new_vel = 0;
new_a = 0;
new_att = 0;
new_av = 0;

if flags(1) ~= mean(y_pos)
    new_pos = 1;
    flags(1) = mean(y_pos);
end
if flags(2) ~= mean(y_a)
    new_a = 1;
    flags(2) = mean(y_a);
end    
if flags(3) ~= mean(y_att)
    new_att = 1;
    flags(3) = mean(y_att);
end    
if flags(4) ~= mean(y_av)
    new_av = 1;
    flags(4) = mean(y_av);
end 
if flags(5) ~= mean(y_vel)
    new_vel = 1;
    flags(5) = mean(y_vel);
end   


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
    Jx      = x(13);
    Jy      = x(14);
    Jz      = x(15);
    b       = x(16); 
    k       = x(17);
    
    % Known parameters
    g = Par.gravity;
    mass = Par.mass;
    d = Par.d ; % m   Lenght arm
    Jxz = 0;
    %----------------------------
    
    w_1 = uu(1);
    w_2 = uu(2);
    w_3 = uu(3);
    w_4 = uu(4);
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
    %----------------------------------
   
    Jxdot = 0;
    Jydot = 0;
    Jzdot = 0;   
    bdot = 0; 
    kdot = 0;
    
    %euler integration
    x(1) = pn + pndot*dt;
    x(2) = pe + pedot*dt;
    x(3) = pd + pddot*dt;
    x(4) = u  + udot*dt;
    x(5) = v  + vdot*dt;
    x(6) = w  + wdot*dt;
    x(7) = phi + phidot*dt;    
    x(8) = theta + thetadot*dt;
    x(9) = psi + psidot*dt;
    x(10) = p + pdot*dt;
    x(11) = q + qdot*dt;
    x(12) = r + rdot*dt;    
    x(13) = Jx + Jxdot*dt;
    x(14) = Jy  + Jydot*dt;
    x(15) = Jz  + Jzdot*dt;
    x(16) = b  + bdot*dt; 
    x(17) = k  + kdot*dt;
    
   % Covariance matrix estimation 
  [Fx Fu] = Jacobians(x,uu,Par);  
    
 % U = diag([.0001 .0001 .0001 .0001]);
  %Q = diag([.01 .01 .01 .01 .01 .01 .01 .01 .01 .01 .01 .01 .0001 .0001 .0001 .001 .0001]);
  Q = diag([.01 .01 .01 .01 .01 .01 .01 .01 .01 .01 .01 .01 .000 .000 .000 .00 .00005]);
  
  %Pdot = Fx*P + P*Fx' + Fu*U*Fu';
  Pdot = Fx*P + P*Fx' + Q;
  
  P = P + Pdot*dt;
  
  
  % Updates
 conf = 'a'; 
  
 %--- p_n, p_e, p_d update
  if (new_pos == 1)&&((conf=='a')||(conf=='b')||(conf=='d') )
     
       R =  diag([(.1)^2 (.1)^2 (.1)^2]'); 
       
       H =  zeros(3,17);
       H(1:3,1:3) = eye(3);    
          
       h = [[x(1)]
            [x(2)]
            [x(3)]];
       
       L = P*H'/((R + H*P*H'));
       
       if isnan(L)
          q = 10; 
       end 
       
       %disp(det(P))
       
       
       P = (eye(17) - L*H)*P;
       
       x = x + L*(y_pos - h); 
     
      %disp(y_pos')
      %disp(h')
  q = 10; 
   
  end
 
 %--- p_d update
  if (new_pos == 1)&&((conf=='f') )
     
       R =  diag([(.1)^2]'); 
       
       H =  zeros(1,17);
       H(3) = eye(1);    
          
       h = x(3);
       
       L = P*H'/((R + H*P*H'));
       
       if isnan(L)
          q = 10; 
       end 
       
       %disp(det(P))
       
       
       P = (eye(17) - L*H)*P;
       
       x = x + L*(y_pos(3) - h); 
     
      %disp(y_pos')
      %disp(h')
  q = 10; 
   
 end 
 
 %--- u , v, w  update  (linear velocity)
 if (new_vel == 1)&&((conf=='a')||(conf=='c')||(conf=='e') )
     
       R =  diag([(.05)^2 (.05)^2 (.05)^2]'); 
       
       H =  zeros(3,17);
       H(1:3,4:6) = eye(3);    
          
       h = [[x(4)]
            [x(5)]
            [x(6)]];
       
       L = P*H'/((R + H*P*H'));
       
       if isnan(L)
          q = 10; 
       end 
       
       %disp(det(P))       
       
       P = (eye(17) - L*H)*P;
       
       x = x + L*(y_vel - h); 
     
      %disp(y_pos')
      %disp(h')
  q = 10; 
   
 end 
 
 % ---- phi, theta, psi  update  (attitude)
 if (new_att == 1)&&((conf=='a')||(conf=='b')||(conf=='c') )
     
       R =  diag([(.05)^2 (.05)^2 (.05)^2]'); 
       
       H =  zeros(3,17);
       H(1:3,7:9) = eye(3);    
          
       h = [[x(7)]
            [x(8)]
            [x(9)]];
       
       L = P*H'/((R + H*P*H'));
       
       if isnan(L)
          q = 10; 
       end 
       
       %disp(det(P))
       
       
       P = (eye(17) - L*H)*P;
       
       x = x + L*(y_att - h); 
     
      %disp(y_pos')
      %disp(h')
  q = 10; 
   
 end
 
 % ----- p, q, r update  (angular velocity)
 if (new_av == 1)
     
       R =  diag([(.05)^2 (.05)^2 (.05)^2]'); 
       
       H =  zeros(3,17);
       H(1:3,10:12) = eye(3);    
          
       h = [[x(10)]
            [x(11)]
            [x(12)]];
       
       L = P*H'/((R + H*P*H'));
       
       if isnan(L)
          q = 10; 
       end 
       
       %disp(det(P))
       
       
       P = (eye(17) - L*H)*P;
       
       x = x + L*(y_av - h); 
     
      %disp(y_pos')
      %disp(h')
  q = 10; 
   
 end
  
  
 
 
 
 
 
 
 %store system vector and covariance matrix
 block.Dwork(2).Data  = reshape(P,17*17,1);
 block.Dwork(1).Data = x;
 
 block.Dwork(3).Data = flags;
 
 
 % calculate jacobians
 function [Fx Fu] = Jacobians(x,uu,Par)

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
    Jx      = x(13);
    Jy      = x(14);
    Jz      = x(15);
    b       = x(16); 
    k       = x(17); 
    
    w_1 = uu(1);
    w_2 = uu(2);
    w_3 = uu(3);
    w_4 = uu(4);
     
    g = Par.gravity;
    mass = Par.mass;
    d = Par.d ; % m   Lenght arm
    Jxz = 0;
 
 Fx =  ...
[[ 0, 0, 0, cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta),   v*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + w*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)), w*cos(phi)*cos(psi)*cos(theta) - u*cos(psi)*sin(theta) + v*cos(psi)*cos(theta)*sin(phi), w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - u*cos(theta)*sin(psi),                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]
[ 0, 0, 0, cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), - v*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - w*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), w*cos(phi)*cos(theta)*sin(psi) - u*sin(psi)*sin(theta) + v*cos(theta)*sin(phi)*sin(psi), w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + u*cos(psi)*cos(theta),                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]
[ 0, 0, 0,         -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta),                                                                 v*cos(phi)*cos(theta) - w*cos(theta)*sin(phi),                          - u*cos(theta) - w*cos(phi)*sin(theta) - v*sin(phi)*sin(theta),                                                                                                                                   0,                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]
[ 0, 0, 0,                   0,                                                r,                                               -q,                                                                                                             0,                                                                           -g*cos(theta),                                                                                                                                   0,                                           0,                                                                                  -w,                                          v,                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]
[ 0, 0, 0,                  -r,                                                0,                                                p,                                                                                         g*cos(phi)*cos(theta),                                                                  -g*sin(phi)*sin(theta),                                                                                                                                   0,                                           w,                                                                                   0,                                         -u,                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]
[ 0, 0, 0,                   q,                                               -p,                                                0,                                                                                        -g*cos(theta)*sin(phi),                                                                  -g*cos(phi)*sin(theta),                                                                                                                                   0,                                          -v,                                                                                   u,                                          0,                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                         -(w_1^2 + w_2^2 + w_3^2 + w_4^2)/mass,                                                      0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                 q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta),                           r*cos(phi)*(tan(theta)^2 + 1) + q*sin(phi)*(tan(theta)^2 + 1),                                                                                                                                   0,                                           1,                                                                 sin(phi)*tan(theta),                        cos(phi)*tan(theta),                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                     - r*cos(phi) - q*sin(phi),                                                                                       0,                                                                                                                                   0,                                           0,                                                                            cos(phi),                                  -sin(phi),                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                             (q*cos(phi))/cos(theta) - (r*sin(phi))/cos(theta),             (r*cos(phi)*sin(theta))/cos(theta)^2 + (q*sin(phi)*sin(theta))/cos(theta)^2,                                                                                                                                   0,                                           0,                                                                 sin(phi)/cos(theta),                        cos(phi)/cos(theta),                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,     -(Jxz*q*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz), (r*(Jxz^2 - Jz*(Jy - Jz)))/(Jxz^2 - Jx*Jz) - (Jxz*p*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz), (q*(Jxz^2 - Jz*(Jy - Jz)))/(Jxz^2 - Jx*Jz),                                                                                                (Jz^2*(b*d*w_1^2 - b*d*w_2^2 - b*d*w_3^2 + b*d*w_4^2))/(Jxz^2 - Jx*Jz)^2 - (Jxz*Jz*(k*w_1^2 + k*w_2^2 - k*w_3^2 - k*w_4^2))/(Jxz^2 - Jx*Jz)^2 - (Jxz*p*q)/(Jxz^2 - Jx*Jz) + (Jz*q*r*(Jxz^2 - Jz*(Jy - Jz)))/(Jxz^2 - Jx*Jz)^2 - (Jxz*Jz*p*q*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz)^2,                                                 (Jxz*p*q)/(Jxz^2 - Jx*Jz) - (Jz*q*r)/(Jxz^2 - Jx*Jz), (b*d*w_1^2 - b*d*w_2^2 - b*d*w_3^2 + b*d*w_4^2)/(Jxz^2 - Jx*Jz) - (Jx*Jxz*(k*w_1^2 + k*w_2^2 - k*w_3^2 - k*w_4^2))/(Jxz^2 - Jx*Jz)^2 + (Jx*Jz*(b*d*w_1^2 - b*d*w_2^2 - b*d*w_3^2 + b*d*w_4^2))/(Jxz^2 - Jx*Jz)^2 - (Jxz*p*q)/(Jxz^2 - Jx*Jz) - (q*r*(Jy - 2*Jz))/(Jxz^2 - Jx*Jz) + (Jx*q*r*(Jxz^2 - Jz*(Jy - Jz)))/(Jxz^2 - Jx*Jz)^2 - (Jx*Jxz*p*q*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz)^2,  (Jz*(d*w_1^2 - d*w_2^2 - d*w_3^2 + d*w_4^2))/(Jxz^2 - Jx*Jz), -(Jxz*(w_1^2 + w_2^2 - w_3^2 - w_4^2))/(Jxz^2 - Jx*Jz)]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,           - (2*Jxz*p)/Jy - (r*(Jx - Jz))/Jy,                                                                                   0,            (2*Jxz*r)/Jy - (p*(Jx - Jz))/Jy,                                                                                                                                                                                                                                                                                                                                                                    -(p*r)/Jy, (Jxz*(p^2 - r^2))/Jy^2 - (b*d*w_1^2 - b*d*w_2^2 + b*d*w_3^2 - b*d*w_4^2)/Jy^2 + (p*r*(Jx - Jz))/Jy^2,                                                                                                                                                                                                                                                                                                                                                                             (p*r)/Jy,                    (d*w_1^2 - d*w_2^2 + d*w_3^2 - d*w_4^2)/Jy,                                                      0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0, -(q*(Jxz^2 + Jx*(Jx - Jy)))/(Jxz^2 - Jx*Jz), (Jxz*r*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz) - (p*(Jxz^2 + Jx*(Jx - Jy)))/(Jxz^2 - Jx*Jz),     (Jxz*q*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz), (Jxz*Jz*(b*d*w_1^2 - b*d*w_2^2 - b*d*w_3^2 + b*d*w_4^2))/(Jxz^2 - Jx*Jz)^2 - (Jx*Jz*(k*w_1^2 + k*w_2^2 - k*w_3^2 - k*w_4^2))/(Jxz^2 - Jx*Jz)^2 - (p*q*(2*Jx - Jy))/(Jxz^2 - Jx*Jz) - (k*w_1^2 + k*w_2^2 - k*w_3^2 - k*w_4^2)/(Jxz^2 - Jx*Jz) + (Jxz*q*r)/(Jxz^2 - Jx*Jz) - (Jz*p*q*(Jxz^2 + Jx*(Jx - Jy)))/(Jxz^2 - Jx*Jz)^2 + (Jxz*Jz*q*r*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz)^2,                                                 (Jx*p*q)/(Jxz^2 - Jx*Jz) - (Jxz*q*r)/(Jxz^2 - Jx*Jz),                                                                                                        (Jx*Jxz*(b*d*w_1^2 - b*d*w_2^2 - b*d*w_3^2 + b*d*w_4^2))/(Jxz^2 - Jx*Jz)^2 - (Jx^2*(k*w_1^2 + k*w_2^2 - k*w_3^2 - k*w_4^2))/(Jxz^2 - Jx*Jz)^2 + (Jxz*q*r)/(Jxz^2 - Jx*Jz) - (Jx*p*q*(Jxz^2 + Jx*(Jx - Jy)))/(Jxz^2 - Jx*Jz)^2 + (Jx*Jxz*q*r*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz)^2, (Jxz*(d*w_1^2 - d*w_2^2 - d*w_3^2 + d*w_4^2))/(Jxz^2 - Jx*Jz),  -(Jx*(w_1^2 + w_2^2 - w_3^2 - w_4^2))/(Jxz^2 - Jx*Jz)]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                    0,                                                             0,                                                      0]];
 
 
 
 
Fu = [...
[                                                              0,                                                                0,                                                              0,                                                              0]
[                                                              0,                                                                0,                                                              0,                                                              0]
[                                                              0,                                                                0,                                                              0,                                                              0]
[                                                              0,                                                                0,                                                              0,                                                              0]
[                                                              0,                                                                0,                                                              0,                                                              0]
[                                                -(2*b*w_1)/mass,                                                  -(2*b*w_2)/mass,                                                -(2*b*w_3)/mass,                                                -(2*b*w_4)/mass]
[                                                              0,                                                                0,                                                              0,                                                              0]
[                                                              0,                                                                0,                                                              0,                                                              0]
[                                                              0,                                                                0,                                                              0,                                                              0]
[ (2*Jz*b*d*w_1)/(Jxz^2 - Jx*Jz) - (2*Jxz*k*w_1)/(Jxz^2 - Jx*Jz), - (2*Jxz*k*w_2)/(Jxz^2 - Jx*Jz) - (2*Jz*b*d*w_2)/(Jxz^2 - Jx*Jz), (2*Jxz*k*w_3)/(Jxz^2 - Jx*Jz) - (2*Jz*b*d*w_3)/(Jxz^2 - Jx*Jz), (2*Jxz*k*w_4)/(Jxz^2 - Jx*Jz) + (2*Jz*b*d*w_4)/(Jxz^2 - Jx*Jz)]
[                                                 (2*b*d*w_1)/Jy,                                                  -(2*b*d*w_2)/Jy,                                                 (2*b*d*w_3)/Jy,                                                -(2*b*d*w_4)/Jy]
[ (2*Jxz*b*d*w_1)/(Jxz^2 - Jx*Jz) - (2*Jx*k*w_1)/(Jxz^2 - Jx*Jz), - (2*Jx*k*w_2)/(Jxz^2 - Jx*Jz) - (2*Jxz*b*d*w_2)/(Jxz^2 - Jx*Jz), (2*Jx*k*w_3)/(Jxz^2 - Jx*Jz) - (2*Jxz*b*d*w_3)/(Jxz^2 - Jx*Jz), (2*Jx*k*w_4)/(Jxz^2 - Jx*Jz) + (2*Jxz*b*d*w_4)/(Jxz^2 - Jx*Jz)]
[                                                              0,                                                                0,                                                              0,                                                              0]
[                                                              0,                                                                0,                                                              0,                                                              0]
[                                                              0,                                                                0,                                                              0,                                                              0]
[                                                              0,                                                                0,                                                              0,                                                              0]
[                                                              0,                                                                0,                                                              0,                                                              0]
]; 
     
     
     
      
      
      