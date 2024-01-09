% linearization

syms pn pe pd u v w phi theta psi p q r l m n T1 T2 T3 T4 T5 T6 T7 T8 Jy fx fy fz mass real


 R_b2e = [[1   sin(phi)*tan(theta)   cos(phi)*tan(theta) ];
             [0        cos(phi)           -sin(phi)         ];
             [0   sin(phi)/cos(theta)   cos(phi)/cos(theta) ]];
         
         
 x_a =  R_b2e*[p q r]';
 
   
 pdot = (T1*p*q - T2*q*r) +  (T3*l + T4*n);
    
 qdot = (T5*p*r - T6*(p^2 - r^2))  +  (1/Jy)*m;
    
 rdot = (T7*p*q  -  T1*q*r  )   +   (T4*l + T8*n);
 
 
 fo = [x_a ; pdot ; qdot; rdot]
 
 
 
 JFx = jacobian(fo, [phi, theta, psi, p, q, r ]);
 
 JFu = jacobian(fo, [l, m, n]);
 

 % evaluate at some point
 phi = 0;
 theta = 0;
 psi = 0;
 p = 0;
 q = 0;
 r = 0;
 T4 = 0; % for quads
  
 subs(JFx)
 
 subs(JFu)
 
 %---------------------------------------------------------
 