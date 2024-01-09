function control_mux_v1(block)
% Level-2 MATLAB file S-Function for limited integrator demo.
%   Copyright 1990-2009 The MathWorks, Inc.
%   $Revision: 1.1.6.2 $ 

  setup(block);
  
%endfunction

function setup(block)
  
  % Register number of dialog parameters   
  %block.NumDialogPrms = 3;

  % Register number of input and output ports
  block.NumInputPorts  = 4;
  block.NumOutputPorts = 4;

  % Setup functional port properties to dynamically
  % inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = true;
 % block.InputPort(1).N
  
  block.InputPort(2).Dimensions        = 1;
  block.InputPort(2).DirectFeedthrough = true;
  
  block.InputPort(3).Dimensions        = 1;
  block.InputPort(3).DirectFeedthrough = true;
  
  block.InputPort(4).Dimensions        = 1;
  block.InputPort(4).DirectFeedthrough = true;
  
  block.OutputPort(1).Dimensions       = 1;
  %block.OutputPort(1).SamplingMode = 'Sample';
  
  block.OutputPort(2).Dimensions       = 1;
  %block.OutputPort(2).SamplingMode = 'Sample';
  
  block.OutputPort(3).Dimensions       = 1;
  %block.OutputPort(3).SamplingMode = 'Sample';
  
  block.OutputPort(4).Dimensions       = 1;
  %block.OutputPort(4).SamplingMode = 'Sample';
  
  %% Set block sample time to continuous
  block.SampleTimes = [-1 0];
  
  %% Setup Dwork
  %block.NumContStates = 6;

  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  %block.RegBlockMethod('Derivatives',             @Derivative);  
  block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);
  
%endfunction
 function SetInputPortSamplingMode(block, idx, fd)
 block.InputPort(idx).SamplingMode = fd;
 block.OutputPort(idx).SamplingMode = fd;

function InitConditions(block)

  % Initialize Dwork
  %block.ContStates.Data = block.DialogPrm(3).Data;
  global P;
  
  
  d = P.d_e ; % m   Lenght arm
  b = P.b_e ; % estimated Lift (thrust) factor, N/rad/s
  k = P.k_e ; % estimated Drag factor,  K N.m/rad/s
        
    
    A = [[  b   b   b    b]
         [-d*b d*b d*b -d*b]
         [d*b -d*b d*b -d*b]
         [ k    k   -k    -k ]]; 
     
     
   P.Mt2v = inv(A);  
  
     q = 10;
  

  
%endfunction

function Output(block)

global P;

Mt2v = P.Mt2v;

T = block.InputPort(1).Data ;  % 
tau_r = block.InputPort(2).Data ;  % 
tau_p =  block.InputPort(3).Data ;
tau_y =  block.InputPort(4).Data ;



  
     
     
%
v = [T tau_r tau_p tau_y]';

    
u2 = Mt2v*v ;% find control signal in format [F1 F2]

% saturate if needed

if u2(1) < 0
  u2(1) = 0;  
end
if u2(1) > 1
  u2(1) = 1;  
end
if u2(2) < 0
  u2(2) = 0;  
end
if u2(2) > 1
  u2(2) = 1;  
end  
if u2(3) < 0
  u2(3) = 0;  
end
if u2(3) > 1
  u2(3) = 1;  
end
if u2(4) < 0
  u2(4) = 0;  
end
if u2(4) > 1
  u2(4) = 1;  
end 


u = u2.^(1/2);

block.OutputPort(1).Data = u(1);
block.OutputPort(2).Data = u(2);
block.OutputPort(3).Data = u(3);
block.OutputPort(4).Data = u(4);

  
%endfunction

function Derivative(block)

   

