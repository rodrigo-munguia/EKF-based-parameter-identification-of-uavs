clear all
close all

addpath('ardupilog-master')

%filename = '..//..//LOGS//19-05-02_20-10-34.bin';  %[708 860]
filename = '..//..//LOGS//19-06-08_10-35-16.bin';  %8 m [610-720]
%filename = '..//..//LOGS//80-01-12_09-19-22_1.bin'; % 6m  670-780

%filename = '19-05-02_20-10-34.bin';

log = Ardupilog(filename);

%log_filtered = log.filterMsgs({'IMU'})

iniT =610;
endT = 720; % 875


sliced_log = log.getSlice([708, 870], 'TimeS');  %708

%sliced_log = log.getSlice([500, 650], 'TimeS');

sliced_log = log.getSlice([iniT, endT], 'TimeS');  %708


log_struct = sliced_log.getStruct();


idx = [1 1 1 1 1];
[flag,time,idx] = get_idx_data(log_struct,idx );
init_time = time;
last_time = init_time;

%--------------------------------------------------------------------------
% x = [pn pe pd u v w phi theta psi p q r Jx Jy Jz b k]
g = 9.8;
mass = 1.2;  %kg
d1 = 0.1995;  %m
d2 = 0.1912;  %m
Jxz = 0;
Par.gravity = g;
Par.mass = mass;
Par.d1 = d1 ; % m   Lenght arm
Par.d2 = d2; % m   Lenght arm


pd_0 = -log_struct.POS.Alt(1);
Roll_0 = deg2rad(log_struct.ATT.Roll(1));
Pitch_0 = deg2rad(log_struct.ATT.Pitch(1)); 
Yaw_0 = deg2rad(log_struct.ATT.Yaw(1));
%Yaw_0 = 0;
GyrX_0 = log_struct.IMU.GyrX(1);
GyrY_0 = log_struct.IMU.GyrY(1); 
GyrZ_0 = log_struct.IMU.GyrZ(1); 

r_i = .27;
h = .11;
Jx_0 = (2/5)*mass*r_i^2;
Jy_0 = (2/5)*mass*r_i^2;
Jz_0 = (2/5)*mass*r_i^2;

Jz_0 = (1/2)*mass*r_i^2;
Jx_0 = (1/12)*mass*(3*r_i^2 + h^2);
Jy_0 = Jx_0;

Jx_0 = .007;
Jy_0 = .007;
Jz_0 = .007;


zero_pwm = 1150; % PWM at zero rotor velocity

pwm_h = 1500-zero_pwm;  %  pwm at hover (hypotheses) 1588
b_ini = (mass*g)/(4*pwm_h);
k_ini = b_ini/30; % to improve


% x = [pn pe pd u v w phi theta psi p q r Jx Jy Jz b k]
%x = [0 0 pd_0 0 0 0 Roll_0 Pitch_0 Yaw_0 0 0 0 Jx_0 Jy_0 Jz_0  .0001 .000001]'; 
x = [0 0 0 0 0 0 Roll_0 Pitch_0 Yaw_0 0 0 0 Jx_0 Jy_0 Jz_0  .0000001 .00000001]'; 

% case 1 (uu not squared)
P = diag([.0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .000000002  .000000002  .0000008  .000000001 .000000005]);

Q = diag([.001 .001 .001 .001 .001 .001 .001 .001 .001 .001 .001 .001 .0000000000000 .0000000000000 .00000000000000 .0000000000001 .00000000000001]);
%---
%P = diag([.0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 eps eps eps  eps eps]);
%Q = diag([.001 .001 .001 .001 .001 .001 .001 .001 .001 .001 .001 .001 .00000000001 .00000000001 .00000001 .000000000001 .000000000001]);
%-------
%P = diag([.0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .0001 .00000001  .00000001 .0000001 .000000001 .000000000001]);
%Q = diag([.001 .001 .001 .001 .001 .001 .001 .001 .001 .001 .001 .001 .0000000000001 .0000000000001 .0000000000001 .0000000000001 .000000000000001]);


x_hist = [];
P_hist = [];
y_att_hist = [];
y_gyr_hist = [];
y_POS_hist = [];
y_vel_hist = [];
y_pwm_hist = [];



while (time < endT-1)  % main loop %762

   
    [flag,time,idx] = get_idx_data(log_struct,idx );
  
  if((time < 730)||(time >750)) 
  %if(1>0)      

    %x(16) = .001; %b
    %x(17) = .0001; %k
    
    
    N = 10;
    if (flag(1) == 1)  % ESC PWM
        
        for(j=1:N)
             dt = time-last_time; % get delta time
            
            dt = .1;

            pn      = x(1);
            pe      = x(2);
            pd      = x(3);
            u       = x(4);
            v       = x(5);
            w       = x(6);
            phi     = x(7);
            %phi = 0;
            theta   = x(8);
            %theta = 0;
            psi     = x(9);
            p       = x(10);
            q       = x(11);
            r       = x(12);
            Jx      = x(13);
            Jy      = x(14);
            Jz      = x(15);
            b       = x(16); 
            k       = x(17); 
            
            %k = .00022;
           % b = .0064;

           if isnan(x)
              www = 10; 
           end 

           pwm_m1 = log_struct.RCOU.C1(idx(1)-1)-zero_pwm;
           pwm_m2 = log_struct.RCOU.C2(idx(1)-1)-zero_pwm; 
           pwm_m3 = log_struct.RCOU.C3(idx(1)-1)-zero_pwm; 
           pwm_m4 = log_struct.RCOU.C4(idx(1)-1)-zero_pwm; 
           
           

           % gravity force expressed in the body frame 
          fg_b = [[ -mass*g*sin(theta) ]
                  [  mass*g*cos(theta)*sin(phi)  ]
                  [  mass*g*cos(theta)*cos(phi)  ]];


           % Quad - X  

          % b = .0068;
          % k = b/30;
            A = [[  b   b   b    b]
                 [-d1*b d1*b d1*b -d1*b]
                 [d2*b -d2*b d2*b -d2*b]
                 [ k    k   -k    -k ]];   


         %Tt = A*[w_1^2 w_2^2 w_3^2 w_4^2]';     
         %uu = [pwm_m1^2 pwm_m2^2 pwm_m3^2 pwm_m4^2]';
         uu = [pwm_m1 pwm_m2 pwm_m3 pwm_m4]';
         
         y_pwm_hist = [y_pwm_hist;[time uu' mean(uu)]];

         Tt = A*uu;


             % propultion force    
            % Eq. (4.18)

            f_b =  fg_b  - [0 0 Tt(1)]';

            if (abs(f_b(3)) > 5)
               www = 10; 
            end    

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
            % udot =  (1/mass)*fx;    
            % vdot =   (1/mass)*fy;  
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
            %----------------------------------

            Jxdot = 0;
            Jydot = 0;
            Jzdot = 0;   
            bdot = 0; 
            kdot = 0;

            %euler integration
            x(1) = pn + pndot*(dt/N);
            x(2) = pe + pedot*(dt/N);
            x(3) = pd + pddot*(dt/N);
            x(4) = u  + udot*(dt/N);
            x(5) = v  + vdot*(dt/N);
            x(6) = w  + wdot*(dt/N);
            x(7) = phi + phidot*(dt/N);    
            x(8) = theta + thetadot*(dt/N);
            x(9) = psi + psidot*(dt/N);

            if (p > 0)
               www = 10; 
            end    

            x(10) = p + pdot*(dt/N);
            x(11) = q + qdot*(dt/N);
            x(12) = r + rdot*(dt/N);    
            x(13) = Jx + Jxdot*(dt/N);
            x(14) = Jy  + Jydot*(dt/N);
            x(15) = Jz  + Jzdot*(dt/N);
            x(16) = b  + bdot*(dt/N); 
            x(17) = k  + kdot*(dt/N);


               % Covariance matrix estimation 
              [Fx Fu] = Jacobians(x,uu,Par);  

            

              %Pdot = Fx*P + P*Fx' + Fu*U*Fu';
              Pdot = Fx*P + P*Fx' + Q;


             %P = Pdot;
              P = P + Pdot*(dt/N);
              
              
               
              
        end
        last_time = time;
       www = 10;
    
    end
    
   
    Gyr_U = 1;
    Att_U = 1;
    Pos_U = 1;
    Vel_U =1;
    
    if (flag(2) == 1)  % IMU data
        
       GyrX = log_struct.IMU.GyrX(idx(2)-1);
       GyrY = log_struct.IMU.GyrY(idx(2)-1); 
       GyrZ =log_struct.IMU.GyrZ(idx(2)-1);
       
       y_av = [GyrX GyrY GyrZ]';
       y_av';
       R =  diag([(.001)^2 (.001)^2 (.001)^2]'); 
       
       H =  zeros(3,17);
       H(1:3,10:12) = eye(3);    
          
       h = [[x(10)]
            [x(11)]
            [x(12)]];
       
       L = P*H'/((R + H*P*H'));
       
       if isnan(L)
          www = 10; 
       end 
       
       %disp(det(P))
       if(Gyr_U ==1)       
           P = (eye(17) - L*H)*P;       
           x = x + L*(y_av - h); 
       end
       y_gyr_hist = [y_gyr_hist;[time y_av']];
        
       www = 10;
     
    end
     
     if (flag(3) == 1)  % Attitude data
        
       Roll = deg2rad(log_struct.ATT.Roll(idx(3)-1));
       Pitch = deg2rad(log_struct.ATT.Pitch(idx(3)-1)); 
       Yaw = deg2rad(log_struct.ATT.Yaw(idx(3)-1)); 
       
       y_att = [Roll Pitch Yaw]';
       y_att';
       
        R =  diag([(.01)^2 (.01)^2 (.01)^2]'); 
       
       H =  zeros(3,17);
       H(1:3,7:9) = eye(3);    
          
       h = [[x(7)]
            [x(8)]
            [x(9)]];
       
       L = P*H'/((R + H*P*H'));
       
       if isnan(L)
          www = 10; 
       end 
       
       %disp(det(P))
       
       if (Att_U == 1)  %update
           P = (eye(17) - L*H)*P;       
           x = x + L*(y_att - h); 
       end
       
       www = 10;
       
       y_att_hist = [y_att_hist;[time y_att']];
     end
    
    
    if (flag(4) == 1)  % Position data
        
       Alt = -log_struct.POS.Alt(idx(4)-1);
       
       y_pos = Alt-pd_0;
       
       R =  diag([(.05)^2]'); 
       
       H =  zeros(1,17);
       H(3) = eye(1);    
          
       h = x(3);
       
       L = P*H'/((R + H*P*H'));
       
       if isnan(L)
          www = 10; 
       end 
       
       %disp(det(P))       
       
       inv_pd = y_pos - h;
       
       
       if (Pos_U == 1)
           P = (eye(17) - L*H)*P;
           x = x + L*(inv_pd);
       end
       %else
           % L*(inv_pd)
          www = 10;
       %end
     
       y_POS_hist = [y_POS_hist;[time y_pos']];
        
       www = 10;
       
      x(3);
    
    end
    
    if (flag(5) == 1)  % vel data
       VN = log_struct.NKF1.VN(idx(5)-1);
       VE = log_struct.NKF1.VE(idx(5)-1); 
       VD = log_struct.NKF1.VD(idx(5)-1);
       phi     = x(7);          
       theta   = x(8);           
       psi     = x(9);
        % body to wold rotation matrix   (check)
            R_b2v = [[cos(theta)*cos(psi)  sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)  cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)];
                     [cos(theta)*sin(psi)  sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)];
                     [-sin(theta)    sin(phi)*cos(theta)   cos(phi)*cos(theta) ]];
        
       y_vel = R_b2v'*[VN VE VD]';
       
        R =  diag([(.1)^2 (.1)^2 (.01)^2]'); 
       
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
       if(Vel_U == 1)
        P = (eye(17) - L*H)*P;       
        x = x + L*(y_vel - h); 
       end
       
       www = 10;
       y_vel_hist = [y_vel_hist;[time y_vel']];
       
    end 
     
    if (flag(5) == 5)  % alt vel data
       VN = log_struct.NKF1.VN(idx(5)-1);
       VE = log_struct.NKF1.VE(idx(5)-1); 
       VD = log_struct.NKF1.VD(idx(5)-1);
       phi     = x(7);          
       theta   = x(8);           
       psi     = x(9);
        % body to wold rotation matrix   (check)
            R_b2v = [[cos(theta)*cos(psi)  sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)  cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)];
                     [cos(theta)*sin(psi)  sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)];
                     [-sin(theta)    sin(phi)*cos(theta)   cos(phi)*cos(theta) ]];
        
       y_vel = R_b2v'*[VN VE VD]';
       
        R =  (1)^2; 
       
       H =  zeros(1,17);
       H(6) = 1;    
          
       h = x(6);
          
       
       L = P*H'/((R + H*P*H'));
       
       if isnan(L)
          q = 10; 
       end 
       
       %disp(det(P))       
       if(Vel_U == 1)
        P = (eye(17) - L*H)*P;       
        x = x + L*(y_vel(3) - h); 
       end
       
       www = 10;
       y_vel_hist = [y_vel_hist;[time y_vel']];
       
    end     
    
   x_hist = [x_hist ; [time  x'] ];
   P_hist = [P_hist ; [time  diag(P)'] ];
   
  end  
end   

figure(1)  %Att
plot(y_att_hist(:,1)-iniT,rad2deg(y_att_hist(:,2)),'color','r') % roll
hold on
%plot(x_hist(:,1),rad2deg(x_hist(:,8)),'color','b') % roll
hold on
%plot(x_hist(:,1),rad2deg(x_hist(:,8))+rad2deg(sqrt(P_hist(:,8)))  ,'color','m') % roll
hold on
plot(y_att_hist(:,1)-iniT,rad2deg(y_att_hist(:,3)),'color','b') %measurements
hold on
%plot(x_hist(:,1),rad2deg(x_hist(:,9)),'color','g') % pitch
hold on
plot(y_att_hist(:,1)-iniT,rad2deg(y_att_hist(:,4))-100,'color','g') %measurements
hold on
%plot(x_hist(:,1),rad2deg(x_hist(:,10)),'color','b') % pitch
axis([0 endT-iniT -50 180])

figure(2)  %p q 
plot(x_hist(:,1)-iniT,x_hist(:,11),'color','r') % p
hold on
plot(x_hist(:,1)-iniT,x_hist(:,12),'color','b') % q
hold on
plot(x_hist(:,1)-iniT,x_hist(:,13),'color','g') % r
axis([0 endT-iniT -3 4]) 

figure(3)  %alt
plot(x_hist(:,1)-iniT,-x_hist(:,4),'color','g') % roll
hold on
%plot(x_hist(:,1),rad2deg(x_hist(:,4))+rad2deg(sqrt(P_hist(:,4)))  ,'color','m') % roll
hold on
%plot(y_POS_hist(:,1), -y_POS_hist(:,2),'color','b') %measurements
axis([0 endT-iniT -2.5 2.5]) 

%{




figure(5)  %p q (vel)
plot(x_hist(:,1),x_hist(:,5),'color','r') % p
hold on
%plot(x_hist(:,1),rad2deg(x_hist(:,11))+rad2deg(sqrt(P_hist(:,11)))  ,'color','m') % roll
hold on
plot(y_gyr_hist(:,1), y_gyr_hist(:,2),'color','b') %measurements
hold on
plot(x_hist(:,1),x_hist(:,6),'color','g') %q
hold on
plot(y_gyr_hist(:,1), y_gyr_hist(:,3),'color','y') 
hold on
%plot(x_hist(:,1),x_hist(:,7),'color','m') %r
hold on
plot(y_gyr_hist(:,1), y_gyr_hist(:,4),'color','k') %measurements


figure(6) %pwm
plot(y_pwm_hist(:,1), y_pwm_hist(:,2),'color','r') 
hold on
plot(y_pwm_hist(:,1), y_pwm_hist(:,3),'color','b') 
hold on
plot(y_pwm_hist(:,1), y_pwm_hist(:,4),'color','g') 
hold on
plot(y_pwm_hist(:,1), y_pwm_hist(:,5),'color','y') 
hold on
plot(y_pwm_hist(:,1), y_pwm_hist(:,6),'color','m') 
www = 10;




%}




figure(7)
plot(x_hist(:,1)-iniT,x_hist(:,17),'color','b') % b
axis([0 endT-iniT 0 7*10^-3])

figure(8)
plot(x_hist(:,1)-iniT,x_hist(:,18),'color','b') % k
axis([0 endT-iniT 0 2*10^-4])


%figure(10)
%plot(y_att_hist(:,1),rad2deg(y_att_hist(:,4)),'color','g') %yaw


figure(9)
plot(x_hist(:,1)-iniT,x_hist(:,14),'color','r') % Jx
hold on
plot(x_hist(:,1)-iniT,x_hist(:,15),'color','b') % Jy
hold on
plot(x_hist(:,1)-iniT,x_hist(:,16),'color','g') %Jz
axis([0 endT-iniT 0 .06])

figure(10)
plot(y_pwm_hist(:,1)-iniT,y_pwm_hist(:,2)+zero_pwm ,'color','r') % pwm_1
hold on
plot(y_pwm_hist(:,1)-iniT,y_pwm_hist(:,3)+zero_pwm ,'color','b') % pwm_2
hold on
plot(y_pwm_hist(:,1)-iniT,y_pwm_hist(:,4)+zero_pwm ,'color','g') % pwm_3
hold on
plot(y_pwm_hist(:,1)-iniT,y_pwm_hist(:,5)+zero_pwm ,'color','m') % pwm_4
axis([0 endT-iniT 1000 1900]) 


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
    d1 = Par.d1 ; % m   Lenght arm
    d2 = Par.d2 ; % m   Lenght arm
    Jxz = 0;
    
    
    
 
Fx =[...
[ 0, 0, 0, cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta),   v*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + w*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)), w*cos(phi)*cos(psi)*cos(theta) - u*cos(psi)*sin(theta) + v*cos(psi)*cos(theta)*sin(phi), w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - u*cos(theta)*sin(psi),                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]
[ 0, 0, 0, cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), - v*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - w*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), w*cos(phi)*cos(theta)*sin(psi) - u*sin(psi)*sin(theta) + v*cos(theta)*sin(phi)*sin(psi), w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + u*cos(psi)*cos(theta),                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]
[ 0, 0, 0,         -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta),                                                                 v*cos(phi)*cos(theta) - w*cos(theta)*sin(phi),                          - u*cos(theta) - w*cos(phi)*sin(theta) - v*sin(phi)*sin(theta),                                                                                                                                   0,                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]
[ 0, 0, 0,                   0,                                                r,                                               -q,                                                                                                             0,                                                                           -g*cos(theta),                                                                                                                                   0,                                           0,                                                                                  -w,                                          v,                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]
[ 0, 0, 0,                  -r,                                                0,                                                p,                                                                                         g*cos(phi)*cos(theta),                                                                  -g*sin(phi)*sin(theta),                                                                                                                                   0,                                           w,                                                                                   0,                                         -u,                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]
[ 0, 0, 0,                   q,                                               -p,                                                0,                                                                                        -g*cos(theta)*sin(phi),                                                                  -g*cos(phi)*sin(theta),                                                                                                                                   0,                                          -v,                                                                                   u,                                          0,                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                             -(w_1 + w_2 + w_3 + w_4)/mass,                                              0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                 q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta),                           r*cos(phi)*(tan(theta)^2 + 1) + q*sin(phi)*(tan(theta)^2 + 1),                                                                                                                                   0,                                           1,                                                                 sin(phi)*tan(theta),                        cos(phi)*tan(theta),                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                     - r*cos(phi) - q*sin(phi),                                                                                       0,                                                                                                                                   0,                                           0,                                                                            cos(phi),                                  -sin(phi),                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                             (q*cos(phi))/cos(theta) - (r*sin(phi))/cos(theta),             (r*cos(phi)*sin(theta))/cos(theta)^2 + (q*sin(phi)*sin(theta))/cos(theta)^2,                                                                                                                                   0,                                           0,                                                                 sin(phi)/cos(theta),                        cos(phi)/cos(theta),                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,     -(Jxz*q*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz), (r*(Jxz^2 - Jz*(Jy - Jz)))/(Jxz^2 - Jx*Jz) - (Jxz*p*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz), (q*(Jxz^2 - Jz*(Jy - Jz)))/(Jxz^2 - Jx*Jz),                                                                                        (Jz^2*(b*d1*w_1 - b*d1*w_2 - b*d1*w_3 + b*d1*w_4))/(Jxz^2 - Jx*Jz)^2 - (Jxz*Jz*(k*w_1 + k*w_2 - k*w_3 - k*w_4))/(Jxz^2 - Jx*Jz)^2 - (Jxz*p*q)/(Jxz^2 - Jx*Jz) + (Jz*q*r*(Jxz^2 - Jz*(Jy - Jz)))/(Jxz^2 - Jx*Jz)^2 - (Jxz*Jz*p*q*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz)^2,                                             (Jxz*p*q)/(Jxz^2 - Jx*Jz) - (Jz*q*r)/(Jxz^2 - Jx*Jz), (b*d1*w_1 - b*d1*w_2 - b*d1*w_3 + b*d1*w_4)/(Jxz^2 - Jx*Jz) + (Jx*Jz*(b*d1*w_1 - b*d1*w_2 - b*d1*w_3 + b*d1*w_4))/(Jxz^2 - Jx*Jz)^2 - (Jx*Jxz*(k*w_1 + k*w_2 - k*w_3 - k*w_4))/(Jxz^2 - Jx*Jz)^2 - (Jxz*p*q)/(Jxz^2 - Jx*Jz) - (q*r*(Jy - 2*Jz))/(Jxz^2 - Jx*Jz) + (Jx*q*r*(Jxz^2 - Jz*(Jy - Jz)))/(Jxz^2 - Jx*Jz)^2 - (Jx*Jxz*p*q*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz)^2,  (Jz*(d1*w_1 - d1*w_2 - d1*w_3 + d1*w_4))/(Jxz^2 - Jx*Jz), -(Jxz*(w_1 + w_2 - w_3 - w_4))/(Jxz^2 - Jx*Jz)]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,           - (2*Jxz*p)/Jy - (r*(Jx - Jz))/Jy,                                                                                   0,            (2*Jxz*r)/Jy - (p*(Jx - Jz))/Jy,                                                                                                                                                                                                                                                                                                                                                -(p*r)/Jy, (Jxz*(p^2 - r^2))/Jy^2 - (b*d2*w_1 - b*d2*w_2 + b*d2*w_3 - b*d2*w_4)/Jy^2 + (p*r*(Jx - Jz))/Jy^2,                                                                                                                                                                                                                                                                                                                                                             (p*r)/Jy,                    (d2*w_1 - d2*w_2 + d2*w_3 - d2*w_4)/Jy,                                              0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0, -(q*(Jxz^2 + Jx*(Jx - Jy)))/(Jxz^2 - Jx*Jz), (Jxz*r*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz) - (p*(Jxz^2 + Jx*(Jx - Jy)))/(Jxz^2 - Jx*Jz),     (Jxz*q*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz), (Jxz*Jz*(b*d1*w_1 - b*d1*w_2 - b*d1*w_3 + b*d1*w_4))/(Jxz^2 - Jx*Jz)^2 - (k*w_1 + k*w_2 - k*w_3 - k*w_4)/(Jxz^2 - Jx*Jz) - (p*q*(2*Jx - Jy))/(Jxz^2 - Jx*Jz) - (Jx*Jz*(k*w_1 + k*w_2 - k*w_3 - k*w_4))/(Jxz^2 - Jx*Jz)^2 + (Jxz*q*r)/(Jxz^2 - Jx*Jz) - (Jz*p*q*(Jxz^2 + Jx*(Jx - Jy)))/(Jxz^2 - Jx*Jz)^2 + (Jxz*Jz*q*r*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz)^2,                                             (Jx*p*q)/(Jxz^2 - Jx*Jz) - (Jxz*q*r)/(Jxz^2 - Jx*Jz),                                                                                                    (Jx*Jxz*(b*d1*w_1 - b*d1*w_2 - b*d1*w_3 + b*d1*w_4))/(Jxz^2 - Jx*Jz)^2 - (Jx^2*(k*w_1 + k*w_2 - k*w_3 - k*w_4))/(Jxz^2 - Jx*Jz)^2 + (Jxz*q*r)/(Jxz^2 - Jx*Jz) - (Jx*p*q*(Jxz^2 + Jx*(Jx - Jy)))/(Jxz^2 - Jx*Jz)^2 + (Jx*Jxz*q*r*(Jx - Jy + Jz))/(Jxz^2 - Jx*Jz)^2, (Jxz*(d1*w_1 - d1*w_2 - d1*w_3 + d1*w_4))/(Jxz^2 - Jx*Jz),  -(Jx*(w_1 + w_2 - w_3 - w_4))/(Jxz^2 - Jx*Jz)]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]
[ 0, 0, 0,                   0,                                                0,                                                0,                                                                                                             0,                                                                                       0,                                                                                                                                   0,                                           0,                                                                                   0,                                          0,                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                    0,                                                         0,                                              0]]; 
 
Fu = [...
[                                                   0,                                                     0,                                                   0,                                                   0]
[                                                   0,                                                     0,                                                   0,                                                   0]
[                                                   0,                                                     0,                                                   0,                                                   0]
[                                                   0,                                                     0,                                                   0,                                                   0]
[                                                   0,                                                     0,                                                   0,                                                   0]
[                                             -b/mass,                                               -b/mass,                                             -b/mass,                                             -b/mass]
[                                                   0,                                                     0,                                                   0,                                                   0]
[                                                   0,                                                     0,                                                   0,                                                   0]
[                                                   0,                                                     0,                                                   0,                                                   0]
[ (Jz*b*d1)/(Jxz^2 - Jx*Jz) - (Jxz*k)/(Jxz^2 - Jx*Jz), - (Jxz*k)/(Jxz^2 - Jx*Jz) - (Jz*b*d1)/(Jxz^2 - Jx*Jz), (Jxz*k)/(Jxz^2 - Jx*Jz) - (Jz*b*d1)/(Jxz^2 - Jx*Jz), (Jxz*k)/(Jxz^2 - Jx*Jz) + (Jz*b*d1)/(Jxz^2 - Jx*Jz)]
[                                           (b*d2)/Jy,                                            -(b*d2)/Jy,                                           (b*d2)/Jy,                                          -(b*d2)/Jy]
[ (Jxz*b*d1)/(Jxz^2 - Jx*Jz) - (Jx*k)/(Jxz^2 - Jx*Jz), - (Jx*k)/(Jxz^2 - Jx*Jz) - (Jxz*b*d1)/(Jxz^2 - Jx*Jz), (Jx*k)/(Jxz^2 - Jx*Jz) - (Jxz*b*d1)/(Jxz^2 - Jx*Jz), (Jx*k)/(Jxz^2 - Jx*Jz) + (Jxz*b*d1)/(Jxz^2 - Jx*Jz)]
[                                                   0,                                                     0,                                                   0,                                                   0]
[                                                   0,                                                     0,                                                   0,                                                   0]
[                                                   0,                                                     0,                                                   0,                                                   0]
[                                                   0,                                                     0,                                                   0,                                                   0]
[                                                   0,                                                     0,                                                   0,                                                   0]
]; 
     
 end     
 
      



% get flag and index of data
function [flag,time,idx] = get_idx_data(log_struct,idx )

    t_esc = log_struct.RCOU.TimeS(idx(1));
    t_imu = log_struct.IMU.TimeS(idx(2));
    t_att = log_struct.ATT.TimeS(idx(3));
    t_pos = log_struct.POS.TimeS(idx(4));
    t_vel = log_struct.NKF1.TimeS(idx(5));

    [mi,li] = min([t_esc t_imu t_att t_pos t_vel]);

    if (li==1)
       idx(1) = idx(1) + 1;
       flag = [1 0 0 0 0];
       time = t_esc;
    elseif (li==2)
       idx(2) = idx(2) + 1;
       flag = [0 1 0 0 0];
       time = t_imu;
    elseif (li==3)
       idx(3) = idx(3) + 1;
       flag = [0 0 1 0 0];
       time = t_att;
    elseif (li==4)
       idx(4) = idx(4) + 1;
       flag = [0 0 0 1 0];
       time = t_pos;
    elseif (li==5)
       idx(5) = idx(5) + 1;
       flag = [0 0 0 0 1];
       time = t_vel;
    end    


end
