function [V,F, patchcolors] = AircraftVFC


fuse_l1 = 1;
fuse_l2 = .5;
fuse_l3 = 3;
fuse_w = .5;
fuse_h = .5;

wing_l = 1;
wing_w = 4;

tailwing_l = .5;
tailwing_w = 1.5;
tail_h = 1;







% Define the vertices
% figure 2.14
V = [...
    fuse_l1    0           0   ; % point 1
    fuse_l2  fuse_w/2 -fuse_h/2; %point 2 
    fuse_l2 -fuse_w/2 -fuse_h/2; %point 3
    fuse_l2 -fuse_w/2  fuse_h/2; %point 4
    fuse_l2  fuse_w/2  fuse_h/2; %point 5
    -fuse_l3   0           0   ; %point 6 
       0     wing_w/2      0   ; %point 7
    -wing_l  wing_w/2      0   ; %point 8
    -wing_l -wing_w/2      0   ; %point 9
       0    -wing_w/2      0   ; %point 10
    (-fuse_l3+tailwing_l) tailwing_w/2 0 ; %point 11
    -fuse_l3  tailwing_w/2 0;  %point 12
    -fuse_l3 -tailwing_w/2 0;  %point 13
    (-fuse_l3+tailwing_l) -tailwing_w/2 0 ; %point 14
    (-fuse_l3+tailwing_l) 0  0 ; %point 15
    -fuse_l3 0 -tail_h  ; %point 16
    ...
    ];

% define faces as a list of vertices numbered above
F = [...
    1 , 2 , 3 ; % nose
    1 , 4 , 5 ; % nose
    1 , 3 , 4 ; % nose
    1 , 2 , 5 ; % nose
    2 , 3 , 6 ; % fuse 
    3 , 4 , 6 ; % fuse
    4 , 5 , 6 ; % fuse
    2 , 5 , 6 ; % fuse   
    7 , 8 , 9 ; % wing
    7 , 9 , 10; % wing
    11, 12 , 13; %H_tailwing
    11 , 13, 14; %H_tailwing
    6 , 15, 16 ; %V_tailwing
    ...   
    ];

% define colors for each face
myred = [1,0,0];
myblack = [0,0,0];
myyellow = [1,1,0];

patchcolors = [...
    myblack;
    myblack;
    myblack;
    myblack;
    myred;
    myred;
    myred;
    myred;
    myyellow;
    myyellow;
    myyellow;
    myyellow;
    myred;
    ...
    ];