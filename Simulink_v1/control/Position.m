function Position(block)
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
 
  block.InputPort(1).Dimensions        = [4 1];
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = [2 1];
  block.InputPort(2).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = [2 1];
  
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
  block.NumDworks                = 2;
  block.Dwork(1).Name            = 'xi_pn'; 
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'xi_pe'; 
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = true;
  
  

function Start(block)

  %% Initialize Dwork

  
  
   block.Dwork(1).Data = 0;  % initialize xi ; integrator error state p_n
   block.Dwork(2).Data = 0;  % initialize xi ; integrator error state p_e
  
  
  
%block.ContStates.Data(1) = PAR.iniSt.thetar_r;

  
  
  
function Output(block)


P = block.DialogPrm(1).Data;  % get parameters
x = block.InputPort(1).Data ; % state feedback

% north control

xi_pn = block.Dwork(1).Data;  %get xi state

xa = [x(1);x(3);xi_pn]; % augmented state



k = P.C.K_pn;
ki = P.C.Ki_pn;

K1 = [k -ki];
%u = Ki*xi - K*xa;
u_pn = -K1*xa; 

%disp(u_pn)
%-----------------
% east control

xi_pe = block.Dwork(2).Data;  %get xi state

xa = [x(2);x(4);xi_pe]; % augmented state

k = P.C.K_pe;
ki = P.C.Ki_pe;

K1 = [k -ki];
%u = Ki*xi - K*xa;
u_pe = -K1*xa; 

%------------------------------------



block.OutputPort(1).Data = [-u_pn u_pe]';

  
%endfunction

function Update(block)

P = block.DialogPrm(1).Data;  % get parameters
dt = P.C.dt;

% update state usd for integrate value

xi_pn = block.Dwork(1).Data;  %get xi state pn
xi_pe = block.Dwork(2).Data;  %get xi state pe


%xi = GVAR.Calt_xi;
C = [1 0]; 
 
x = block.InputPort(1).Data ; % state feedback
r =  block.InputPort(2).Data ; % reference input


dot_xi_pn =  x(1) -r(1);
dot_xi_pe =  x(2) -r(2);



xi_pn = xi_pn + dot_xi_pn*dt;  % integrate state
xi_pe = xi_pe + dot_xi_pe*dt;  % integrate state


%disp(xi_r);

block.Dwork(1).Data = xi_pn;
block.Dwork(2).Data = xi_pe;




