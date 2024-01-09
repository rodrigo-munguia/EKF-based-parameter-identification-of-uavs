P.gravity = 9.8;
   
%physical parameters of airframe
P.mass = 1.56;
P.Jx   = 0.1147;
P.Jy   = 0.0576;
P.Jz   = 0.1712;
P.Jxz  = 0.0015;

% aerodynamic coefficients
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;
P.rho           = 1.2682;
P.c             = 0.3302;
P.b             = 1.4224;
P.S_wing        = 0.2589;
P.S_prop        = 0.0314;
P.k_motor       = 20;
P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_M_0         = 0.0;
P.C_M_alpha     = -0.38;
P.C_M_q         = -3.6;
P.C_M_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = -0.26;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1;


% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_wx = 1250;
P.L_wy = 1750;
P.L_wz = 1750;
P.sigma_wx = 1; 
P.sigma_wy = 1;
P.sigma_wz = 1;
P.Va0 = 10;

% autopilot sample rate
P.Ts = 0.01;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.Va; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

