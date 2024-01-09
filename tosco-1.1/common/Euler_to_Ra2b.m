function [Ra2b] = Euler_to_Ra2b (roll,pitch,yaw)
%**************************************************************************
% Rodrigo Munguía 2012
% Function for transforming from EULER angles to rotation matrix (Ra2b)
% The rotation is defined by the sequence of rotations (of the body respect to the global reference)
%:  yaw -> pitch -> roll  
%   
% Body to world rotation matrix "Rb2w" is obtained if the orientation of the body are determined by (euler angles) extrinsic rotations
% [Rb2w] = Euler_to_Rb2w (roll,pitch,yaw)

% World to body rotation matrix "Rw2b" is obtained if the orientation of the body are determined by (euler angles) intrinsic rotations
% [Rw2b] = Euler_to_Rb2w (roll,pitch,yaw) 

% world (w) =  navigation (n)
% body (n) = robot (r)

%**************************************************************************


phi = roll;    %Euler angles
theta = pitch;
%theta = 1;
psi = yaw;


Ra2b = [[          cos(psi)*cos(theta)                       sin(psi)*cos(theta)                            -sin(theta)        ];
            [-sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi) cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi) cos(theta)*sin(phi)];
            [sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi) -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi) cos(theta)*cos(phi)]];
			
			
%Ra2b = Ra2b';
