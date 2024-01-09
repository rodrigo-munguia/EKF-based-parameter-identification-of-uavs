function plotMAVStateVariables(uu)
%
% modified 12/11/2009 - RB

    % process inputs to function
    pn          = uu(1);             % North position (meters)
    pe          = uu(2);             % East position (meters)
    h           = -uu(3);            % altitude (meters)
    u           = uu(4);             % body velocity along x-axis (meters/s)
    v           = uu(5);             % body velocity along y-axis (meters/s)
    w           = uu(6);             % body velocity along z-axis (meters/s)
    phi         = 180/pi*uu(7);      % roll angle (degrees)   
    theta       = 180/pi*uu(8);      % pitch angle (degrees)
    psi         = 180/pi*uu(9);             % yaw angle (degrees)
    p           = 180/pi*uu(10);     % body angular rate along x-axis (degrees/s)
    q           = 180/pi*uu(11);     % body angular rate along y-axis (degrees/s)
    r           = 180/pi*uu(12);     % body angular rate along z-axis (degrees/s)
    Va          = uu(13);            % airspeed (m/s)
    alpha       = 180/pi*uu(14);     % angle of attack (degrees)
    beta        = 180/pi*uu(15);     % side slip angle (degrees)
    wn          = uu(16);            % wind in the North direction
    we          = uu(17);            % wind in the East direction
    wd          = uu(18);            % wind in the Down direction
    pn_c        = uu(19);            % commanded North position (meters)
    pe_c        = uu(20);            % commanded East position (meters)
    h_c         = uu(21);            % commanded altitude (meters)
    Va_c        = uu(22);            % commanded airspeed (meters/s)
    alpha_c     = 180/pi*uu(23);     % commanded angle of attack (degrees)
    beta_c      = 180/pi*uu(24);     % commanded side slip angle (degrees)
    phi_c       = 180/pi*uu(25);     % commanded roll angle (degrees)   
    theta_c     = 180/pi*uu(26);     % commanded pitch angle (degrees)
    chi_c       = 180/pi*uu(27);     % commanded course (degrees)
    p_c         = 180/pi*uu(28);     % commanded body angular rate along x-axis (degrees/s)
    q_c         = 180/pi*uu(29);     % commanded body angular rate along y-axis (degrees/s)
    r_c         = 180/pi*uu(30);     % commanded body angular rate along z-axis (degrees/s)
    pn_hat      = uu(31);            % estimated North position (meters)
    pe_hat      = uu(32);            % estimated East position (meters)
    h_hat       = uu(33);            % estimated altitude (meters)
    Va_hat      = uu(34);            % estimated airspeed (meters/s)
    alpha_hat   = 180/pi*uu(35);     % estimated angle of attack (degrees)
    beta_hat    = 180/pi*uu(36);     % estimated side slip angle (degrees)
    phi_hat     = 180/pi*uu(37);     % estimated roll angle (degrees)   
    theta_hat   = 180/pi*uu(38);     % estimated pitch angle (degrees)
    chi_hat     = 180/pi*uu(39);     % estimated course (degrees)
    p_hat       = 180/pi*uu(40);     % estimated body angular rate along x-axis (degrees/s)
    q_hat       = 180/pi*uu(41);     % estimated body angular rate along y-axis (degrees/s)
    r_hat       = 180/pi*uu(42);     % estimated body angular rate along z-axis (degrees/s)
    Vg_hat      = uu(43);            % estimated groundspeed
    wn_hat      = uu(44);            % estimated North wind
    we_hat      = uu(45);            % estimated East wind
    psi_hat     = 180/pi*uu(46);     % estimated heading
    delta_e     = 180/pi*uu(47);     % elevator angle (degrees)
    delta_a     = 180/pi*uu(48);     % aileron angle (degrees)
    delta_r     = 180/pi*uu(49);     % rudder angle (degrees)
    delta_t     = uu(50);            % throttle setting (unitless)
    t           = uu(51);            % simulation time
    
    % compute course angle
    chi = 180/pi*atan2(Va*sin(psi)+we, Va*cos(psi)+wn);

    % define persistent variables 
    persistent pn_handle
    persistent pe_handle
    persistent h_handle
    persistent Va_handle
    persistent alpha_handle
    persistent beta_handle
    persistent phi_handle
    persistent theta_handle
    persistent chi_handle
    persistent p_handle
    persistent q_handle
    persistent r_handle
    persistent delta_e_handle
    persistent delta_a_handle
    persistent delta_r_handle
    persistent delta_t_handle
    persistent psi_handle
    

  % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(2), clf

        subplot(3,2,1)
        hold on
        pn_handle = graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n', []);
        
        
        subplot(3,2,3)
        hold on
        pe_handle = graph_y_yhat_yd(t, pe, pe_hat, pe_c, 'p_e', []);

       
        subplot(3,2,5)
        hold on
        h_handle = graph_y_yhat_yd(t, h, h_hat, h_c, 'h', []);

      

        subplot(3,2,2)
        hold on
        phi_handle = graph_y_yhat_yd(t, phi, phi_hat, phi_c, '\phi', []);
        
      
        
        subplot(3,2,4)
        hold on
        theta_handle = graph_y_yhat_yd(t, theta, theta_hat, theta_c, '\theta', []);
        
       
        
        subplot(3,2,6)
        hold on
        psi_handle = graph_y_yhat_yd(t, psi, psi_hat, 0, '\psi', []);
       
        
    % at every other time step, redraw state variables
    else 
       graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n', pn_handle);
       graph_y_yhat_yd(t, pe, pe_hat, pe_c, 'p_e', pe_handle);
       graph_y_yhat_yd(t, h, h_hat, h_c, 'h', h_handle);       
       graph_y_yhat_yd(t, phi, phi_hat, phi_c, '\phi', phi_handle);
       graph_y_yhat_yd(t, theta, theta_hat, theta_c, '\theta', theta_handle);
       graph_y_yhat_yd(t, psi, psi_hat, 0, '\psi', psi_handle);      
    end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, y, handle, color)
  
  if isempty(handle),
    handle    = plot(t,y,color);
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)
  
  if isempty(handle),
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'g--');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
  end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its 
% desired value yd in red, lab is the label on the graph
function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)
  
  if isempty(handle),
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yhat,'g--');
    handle(3)   = plot(t,yd,'r-.');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
    %drawnow
  end

%
%=============================================================================
% sat
% saturates the input between high and low
%=============================================================================
%
function out=sat(in, low, high)

  if in < low,
      out = low;
  elseif in > high,
      out = high;
  else
      out = in;
  end

% end sat  


