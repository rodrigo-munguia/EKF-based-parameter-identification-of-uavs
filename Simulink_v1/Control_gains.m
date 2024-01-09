% Estimate Control Gains
% run first parameters.m

g = P.gravity;
m = P.mass;  

% Altitude control

A = [[0 1]
     [0 0]];

B = [0 -1/m]';

C = [1 0];

% Augmented integral system

A1 = [[A zeros(2,1)]
      [C 0]];
  
B1 = [B ;0 ];
  
Br = [0 0 1]';
    
eig = [-2 -2 -2];

K1 = acker(A1,B1,eig);

disp('Altitude Gains:');
K = K1(1:2)
Ki = -K1(3)

%--------------------------------------------

Jx = P.Jx;
Jy = P.Jy; 
Jz = P.Jz;
% roll and pitch control

A = [[0 1]
     [0 0]];

B = [0 1/Jx]';

C = [1 0];

% Augmented integral system

A1 = [[A zeros(2,1)]
      [C 0]];
  
B1 = [B ;0 ];
  
Br = [0 0 1]';
    
eig = [-5 -5 -5];

K1 = acker(A1,B1,eig);

disp('Rolland Pitch Gains:');
K = K1(1:2)
Ki = -K1(3)

%--------------------------------------------
% Yaw

% roll and pitch control

A = [[0 1]
     [0 0]];

B = [0 1/Jz]';

C = [1 0];

% Augmented integral system

A1 = [[A zeros(2,1)]
      [C 0]];
  
B1 = [B ;0 ];
  
Br = [0 0 1]';
    
eig = [-5 -5 -5];

K1 = acker(A1,B1,eig);

disp('Yaw Gains:');
K = K1(1:2)
Ki = -K1(3)

q = 10;
%-----------------------------------------------------

% POsition control

A = [[0 1]
     [0 0]];

B = [0 1/m]';

C = [1 0];

% Augmented integral system

A1 = [[A zeros(2,1)]
      [C 0]];
  
B1 = [B ;0 ];
  
Br = [0 0 1]';
    
eig = [-.7 -.7 -.7];

K1 = acker(A1,B1,eig);

disp('Position n-e Gains:');
K = K1(1:2)
Ki = -K1(3)


