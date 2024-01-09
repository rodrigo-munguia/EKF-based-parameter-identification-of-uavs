function Orientation(block)
%************************************
%  Implements a (DISCRETE) proportional-integral controler for attitude


% Level-2 MATLAB file S-Function for limited integrator demo.
%   Copyright 1990-2009 The MathWorks, Inc.
%   $Revision: 1.1.6.2 $ 

  setup(block);
  
%endfunction

function setup(block)
  %global PAR;
  %% Register number of dialog parameters   
  block.NumDialogPrms = 1;

  %% Register number of input and output ports
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = [6 1];
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = [3 1];
  block.InputPort(2).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = [3 1];
  
  %% Set block sample time to continuous
  %block.SampleTimes = [PAR.Calt_Ts  0];
  P = block.DialogPrm(1).Data;  % get parameters
  dt = P.C.dt;
  
  block.SampleTimes = [dt  0];
  
  %% Setup Dwork
  block.NumContStates = 0;  % the integral states 

  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('Start',                   @Start);
  block.RegBlockMethod('Outputs',                 @Output);  
  %block.RegBlockMethod('Derivatives',             @Derivative);  
  block.RegBlockMethod('Update',                  @Update);
  block.RegBlockMethod('PostPropagationSetup',  @PpropagationS);
  
%endfunction
function PpropagationS(block)

 % Setup Dwork
  block.NumDworks                = 3;
  block.Dwork(1).Name            = 'xi_r'; 
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'xi_p'; 
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'xp_y'; 
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;
  block.Dwork(3).Complexity      = 'Real';
  block.Dwork(3).UsedAsDiscState = true;

function Start(block)

  %% Initialize Dwork

  
  
   block.Dwork(1).Data = 0;  % initialize xi ; integrator error state roll
   block.Dwork(2).Data = 0;  % initialize xi ; integrator error state pitch
   block.Dwork(3).Data = 0;  % initialize xi ; integrator error state yaw
  
  
%block.ContStates.Data(1) = PAR.iniSt.thetar_r;

  
  
  
function Output(block)


P = block.DialogPrm(1).Data;  % get parameters
x = block.InputPort(1).Data ; % state feedback

% Roll control

xi_r = block.Dwork(1).Data;  %get xi state

xa = [x(1);x(4);xi_r]; % augmented state

k = P.C.K_r;
ki = P.C.Ki_r;

K1 = [k -ki];
%u = Ki*xi - K*xa;
u_r = -K1*xa; 

%-----------------
% Pitch

xi_p = block.Dwork(2).Data;  %get xi state

xa = [x(2);x(5);xi_p]; % augmented state

k = P.C.K_p;
ki = P.C.Ki_p;

K1 = [k -ki];
%u = Ki*xi - K*xa;
u_p = -K1*xa; 

%------------------------------------
% Yaw
xi_y = block.Dwork(3).Data;  %get xi state

xa = [x(3);x(6);xi_y]; % augmented state

k = P.C.K_y;
ki = P.C.Ki_y;

K1 = [k -ki];
%u = Ki*xi - K*xa;
u_y = -K1*xa; 


block.OutputPort(1).Data = [u_r u_p u_y]';

  
%endfunction

function Update(block)

P = block.DialogPrm(1).Data;  % get parameters
dt = P.C.dt;

% update state usd for integrate value

xi_r = block.Dwork(1).Data;  %get xi state roll
xi_p = block.Dwork(2).Data;  %get xi state pitch
xi_y = block.Dwork(3).Data;  %get xi state yaw

%xi = GVAR.Calt_xi;
C = [1 0]; 
 
x = block.InputPort(1).Data ; % state feedback
r =  block.InputPort(2).Data ; % reference input


dot_xi_r =  x(1) -r(1);
dot_xi_p =  x(2) -r(2);
dot_xi_y = x(3) -r(3);

xi_r = xi_r + dot_xi_r*dt;  % integrate state
xi_p = xi_p + dot_xi_p*dt;  % integrate state
xi_y = xi_y + dot_xi_y*dt;  % integrate state


%disp(xi_r);

block.Dwork(1).Data = xi_r;
block.Dwork(2).Data = xi_p;
block.Dwork(3).Data = xi_y;



