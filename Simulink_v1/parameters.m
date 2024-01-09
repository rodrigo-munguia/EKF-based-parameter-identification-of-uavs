run ../tosco-1.1/startup_tosco
addpath('control') 

global P;


P.gravity = 9.8; % m/s2

%----------------------------------
% Parameters for Quad
%

P.mass = .65;  %kg

%Actual values
% inertia matrix
P.Jx   = .03; %0.0224; %kg.m2
P.Jy   = .035; %0.0224; %kg.m2
P.Jz   = .045; %0.0436;  %kg.m2
P.Jxz  = 0;  %kg.m2

P.d = 0.165;  % m   Lenght arm
P.b = 3.5; % N/rad/s Lift (thrust) factor
P.k = .06; %K N.m/rad/s Drag factor

% Estimated parameters
P.d_e = 0.165;  % m   Lenght arm
P.b_e = 3.5; % N/rad/s Lift (thrust) factor
P.k_e = .06; % .064 K N.m/rad/s Drag factor

%----------------------------------

% Control Gains

P.C.dt = .005; % control time step

P.C.K_alt = [  -7.8000   -3.9000]; % Altitud control proportional gain
P.C.Ki_alt =  5.2000;  % altitude control integral gain

P.C.K_r = [1.6800    0.3360]; % Roll control proportional gain
P.C.Ki_r =   -2.8000;  % Roll control integral gain

P.C.K_p = [1.6800    0.3360]; % Pitch control proportional gain
P.C.Ki_p =   -2.8000;  % Pitch control integral gain

P.C.K_y = [3.2700    0.6540]; % Pitch control proportional gain
P.C.Ki_y =   -5.4500;  % Pitch control integral gain

P.C.K_pn = [ 0.9555    1.3650]; % North control proportional gain
P.C.Ki_pn =   -0.3;   % North control integral gain

P.C.K_pe = [ 0.9555    1.3650]; % North control proportional gain
P.C.Ki_pe =    -0.3;  % North control integral gain


%--------------------------------------
% Sensors
% GPS
P.Ts_gps = .1; % sample rate of GPS in s
%PAR.P.K_gps = 1/16000; % 1/s
%PAR.P.K_gps = 1/1100; % 1/s
P.K_gps = 2; % 1/s
P.sigma_n_gps =  .1; %.21;
P.sigma_e_gps =  .1 %.21; 
P.sigma_h_gps =  .2 %.40;
P.sigma_Vg_gps = 1;
P.sigma_course_gps = 5*pi/180;

% IMU
%P.IMUnoise = [0.0000001 0.0000001 0.0000001 0.0000000005 0.0000000005 0.0000000005];
%P.IMUnoise =  [0.001  0.001  0.001  0.0001  0.0001  0.0001];
P.IMUnoise =  [0.000001  0.000001  0.000001  0.00000005  0.00000005  0.00000005];
%P.IMUnoise =  [0.000001  0.000001  0.000001   0.0000000076  0.0000000076  0.0000000076];

%P.IMUAccel_bias = [.01 -0.01 0.01];
P.IMUAccel_bias = [0 0 0];

% AHRS
%P.zAttNoise =  diag([((pi/180)*.2)^2 ((pi/180)*.2)^2 ((pi/180)*.5)^2]'); %Standard deviations for the True Noise for attitude measurements (degrees)
P.zAttNoise =  [((pi/180)*.2)^2 ((pi/180)*.2)^2 ((pi/180)*.5)^2]'; 
%P.Ts_AHRS = .01;   % (seconds)

%---------------------------------------
% Barometer
P.Ts_Barometer = .01;
P.sigma_static_pres = 1; %0.01e3; %100; % standard deviation of static pressure sensor in Kilo Pascals; MS5611-01BA03 datasheet

% GPS Filter
dt = .01;
P.GPSsm.dt = dt;

%------------------------------------------------
% Parameter estimator

P.est_dt = .01; % sample time





%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 13.5;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;
%P.Jxz  = 0;
% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;
P.eo            = 0.9;    % Oswald factor

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
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
%P.C_n_p         = 0.00;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}
%{
%physical parameters of airframe
P.mass = 1.56;
P.Jx   = 0.1147;
P.Jy   = 0.0576;
P.Jz   = 0.1712;
P.Jxz  = 0.0015;

% aerodynamic coefficients
P.M             = 50; %constan eq 4.10
P.epsilon       = 0.1592;
P.alpha0        = 0.4712; % cutoff angle of stall, eq 4.10
P.rho           = 1.2682; % air density
P.c             = 0.3302; % mean chord of the MAV wing
P.b             = 1.4224; % wingspan 
P.S_wing        = 0.2589; % area of the MAV wing
P.S_prop        = 0.0314;
P.k_motor       = 20;
P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_p         = 0.0254; % parasitic drag coefficient 
P.eo            = 0.9;    % Oswald factor
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

%}
% wind parameters
P.wind_n = 0.5;%3;
P.wind_e = 0;%2;  %m/s
P.wind_d = 0;
P.L_wx = 200;
P.L_wy = 200;
P.L_wz = 50;
P.sigma_wx = 1.06; 
P.sigma_wy = 1.06;
P.sigma_wz = 0.7;
P.Va0 = 10;  %m/s

% autopilot sample rate
P.Ts = 0.01;

P.Va = 5;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = 0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate





