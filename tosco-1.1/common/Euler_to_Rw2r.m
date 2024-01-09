function [Ra2b] = Euler_to_Rw2r (roll,pitch,yaw)
%**************************************************************************
% Rodrigo Munguía 2012
% Function for transforming from EULER angles to rotation matrix (Ra2b)
% The rotation is defined by the sequence of rotations (of the body respect to the global reference)
%:  yaw -> pitch -> roll  
%   
% If  [Rb2w] = Euler_to_Rb2w (roll,pitch,yaw) THEN the roll, pitch and yaw determines the orientation of the body by mean of extrinsic rotations
%
% If  [Rw2b] = Euler_to_Rb2w (roll,pitch,yaw) THEN roll, pitch and yaw determines the orientation of the body by mean of  intrinsic rotations   


%**************************************************************************


phi = roll;    %Euler angles
theta = pitch;
psi = yaw;


Ra2b = [[          cos(psi)*cos(theta)                       sin(psi)*cos(theta)                            -sin(theta)        ];
            [-sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi) cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi) cos(theta)*sin(phi)];
            [sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi) -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi) cos(theta)*cos(phi)]];
			
			
			
