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

function out = forces_moments(x, delta, wind, P)

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
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % compute wind vector in the inertial frame
    w_n = 0;
    w_e = 0;
    w_d = 0;
    
    
    % compute airspeed Va, angle-of-attack alpha, side-slip beta
    Va    = 0;
    alpha = 0;
    beta  = 0;

    Force = [0; 0; 0];
    Torque = [0; 0; 0];
    
    out = [Force; Torque; Va; alpha; beta; w_n; w_e; w_d];

end



