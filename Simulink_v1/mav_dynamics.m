function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,mass,Jx,Jy,Jz,Jxz,pn0,pe0,pd0,u0,v0,w0,phi0,theta0,psi0,p0,q0,r0)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(pn0,pe0,pd0,u0,v0,w0,phi0,theta0,psi0,p0,q0,r0);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,mass,Jx,Jy,Jz,Jxz);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(pn0,pe0,pd0,u0,v0,w0,phi0,theta0,psi0,p0,q0,r0)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [pn0;pe0;pd0;u0;v0;w0;phi0;theta0;psi0;p0;q0;r0];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, mass,Jx,Jy,Jz,Jxz)

    pn    = x(1);
    pe    = x(2);
    pe    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
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
    
   % udot =  (1/mass)*fx;
    
   % vdot =  (1/mass)*fy; 
    
   % wdot =  (1/mass)*fz;
    
   
    
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
        

sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
