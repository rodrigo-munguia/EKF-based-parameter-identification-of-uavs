 syms pn pe pd u v w phi theta psi p q r l m n T1 T2 T3 T4 T5 T6 T7 T8 Jy fx fy fz mass real


   R_r2w = [[cos(theta)*cos(psi)  sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)  cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)];
             [cos(theta)*sin(psi)  sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)];
             [-sin(theta)    sin(phi)*cos(theta)   cos(phi)*cos(theta) ]];
 
         
  x_v =  R_b2e*[u v w]';      
  
  udot = (r*v - q*w) +  (1/mass)*fx;
    
  vdot = (p*w - r*u) +  (1/mass)*fy; 
    
  wdot = (q*u - p*v) +  (1/mass)*fz;
         
  fp = [x_v ; udot ; vdot; wdot];
 
  JFx = jacobian(fp, [pn, pe, pd, u, v, w,  ]);
 
 JFu = jacobian(fp, [fx, fy, fz]);
 
 
  % evaluate at some point
 phi = 0;
 theta = 0;
 psi = 0;
 p = 0;
 q = 0;
 r = 0;

  
 subs(JFx)
 
 subs(JFu)