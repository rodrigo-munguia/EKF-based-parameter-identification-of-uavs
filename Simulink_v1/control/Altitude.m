function Altitude(block)
%************************************
%  Implements a proportional-integral controler for Altitude


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
 
  block.InputPort(1).Dimensions        = [2 1];
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = [1 1];
  block.InputPort(2).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = [1 1];
  
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
  block.NumDworks                = 1;
  block.Dwork(1).Name            = 'x0'; 
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;


function Start(block)

  %% Initialize Dwork

  
  
   block.Dwork(1).Data = 0;  % initialize xi ; integrator state
  
  
%block.ContStates.Data(1) = PAR.iniSt.thetar_r;

  
  
  
function Output(block)


P = block.DialogPrm(1).Data;  % get parameters
g = P.gravity;
m = P.mass;  
k = P.C.K_alt;
ki = P.C.Ki_alt;

% Attitude control

x = block.InputPort(1).Data ; % state feedback
xi = block.Dwork(1).Data;  %get xi state

xa = [x;xi]; % augmented state

K1 = [k -ki];
%u = Ki*xi - K*xa;
u = -K1*xa; 

%disp(u)

% Feedforward control for counteracting gravity

u = u + m*g;


block.OutputPort(1).Data = u;

  
%endfunction

function Update(block)

P = block.DialogPrm(1).Data;  % get parameters
dt = P.C.dt;

% update state usd for integrate value

xi = block.Dwork(1).Data;  %get xi state

%xi = GVAR.Calt_xi;
C = [1 0]; 
 
x = block.InputPort(1).Data ; % state feedback
r =  block.InputPort(2).Data ; % reference input


dot_xi = C*x +r;

xi = xi + dot_xi*dt;  % integrate state

%disp(x(1));




block.Dwork(1).Data = xi;



