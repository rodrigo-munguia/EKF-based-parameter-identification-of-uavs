
P.Jx   = .03; %0.0224; %kg.m2
P.Jy   = .025; %0.0224; %kg.m2
P.Jz   = .045; %0.0436;  %kg.m2


P.d = 0.165;  % m   Lenght arm
P.b = 3.5; % N/rad/s Lift (thrust) factor
P.k = .06; %K N.m/rad/s Drag factor


t_init = 35;
t_final = 40;

b_m = getsampleusingtime(b_hist,t_init,t_final);
k_m = getsampleusingtime(k_hist,t_init,t_final);
Jx_m = getsampleusingtime(Jx_hist,t_init,t_final);
Jy_m = getsampleusingtime(Jy_hist,t_init,t_final);
Jz_m = getsampleusingtime(Jz_hist,t_init,t_final);

b_est = mean(b_m.data);
b_err = norm(b_est - P.b);

k_est = mean(k_m.data);
k_err = norm(k_est - P.k);

Jx_est = mean(Jx_m.data);
Jx_err = norm(Jx_est - P.Jx);

Jy_est = mean(Jy_m.data);
Jy_err = norm(Jy_est - P.Jy);

Jz_est = mean(Jz_m.data);
Jz_err = norm(Jz_est - P.Jz);



fprintf('b =  %f e =, %f \n',b_est,b_err);
fprintf('k = %f  e= %f \n',k_est,k_err);
fprintf('Jx = %f  e= %f \n',Jx_est,Jx_err);
fprintf('Jy = %f  e= %f \n',Jy_est,Jy_err);
fprintf('Jz = %f  e= %f \n',Jz_est,Jz_err);


[b_est,b_err,k_est,k_err,Jx_est,Jx_err,Jy_est,Jy_err,Jz_est,Jz_err]
