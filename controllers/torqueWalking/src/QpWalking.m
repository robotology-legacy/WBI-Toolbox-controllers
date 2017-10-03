%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @authors: Daniele Pucci & Marie Charbonneau
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function QpWalking(block)

setup(block);

function setup(block)
    
block.NumInputPorts  = 6; 
block.NumOutputPorts = 4; 

% Setup port properties to be  dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;


% Definition of port sizes for QP 2 feet
%block.InputPort(1).Dimensions        = -1;  It does not compile if the
%input port dimension is dynamic and the input is a matrix. Leave it
%commented

% Override output port properties
block.OutputPort(1).Dimensions       = block.DialogPrm(1).Data; % f0 Two Feet
block.OutputPort(2).Dimensions       = 6;                       % Exit flag QP 2 Feet
block.OutputPort(3).Dimensions       = 6;                       % f0 One foot     
block.OutputPort(4).Dimensions       = 1;                       % f0 One foot     

for i=1:block.NumInputPorts
    block.InputPort(i).DatatypeID  = -1;          % 'inherited', see http://www.mathworks.com/help/simulink/slref/simulink.blockdata.html#f29-108672
    block.InputPort(i).Complexity  = 'Real';
    block.InputPort(i).DirectFeedthrough = true;
end

for i =1:block.NumOutputPorts
    block.OutputPort(i).DatatypeID  = 0; % double
    block.OutputPort(i).Complexity  = 'Real';
end



% Register parameters
block.NumDialogPrms     = 3;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [-1 0];


% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

% block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
% block.RegBlockMethod('InitializeConditions', @InitializeConditions);
% block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);

block.RegBlockMethod('Outputs', @Outputs);     % Required
% block.RegBlockMethod('Update', @Update);
% block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required    
block.RegBlockMethod('SetInputPortDimensions', @SetInputPortDimensions);
block.RegBlockMethod('SetOutputPortDimensions',@SetOutputPortDimensions);

%end setup

function SetInputPortSamplingMode(block, idx, fd)
 block.InputPort(idx).SamplingMode = fd;

 for i=1:block.NumOutputPorts
    block.OutputPort(i).SamplingMode = fd;
 end

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C-Mex counterpart: mdlSetWorkWidths
%%

%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is 
%%                      present in an enabled subsystem configured to reset 
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C-MEX counterpart: mdlInitializeConditions
%%
% function InitializeConditions(block)


% end InitializeConditions


%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C-MEX counterpart: mdlStart
%%
% function Start(block)
% 
% block.Dwork(1).Data = 0;

%endfunction

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C-MEX counterpart: mdlOutputs
%%

function Outputs(block)
    
    ROBOT_DOF                  = block.DialogPrm(1).Data;
    Ts                         = block.DialogPrm(2).Data;
    constantMaxTolerance       = block.DialogPrm(3).Data;
    
    HessianMatrixQP                 = block.InputPort(1).Data;
    biasVectorQP                    = block.InputPort(2).Data;
    feetAccelerationConstraintMatrix= block.InputPort(3).Data;
    feetAccelerationConstraintUpperBoundVector = block.InputPort(4).Data;
    frictionConeConstraintMatrix    = block.InputPort(5).Data;
    frictionConeUpperBoundVector    = block.InputPort(6).Data;
        
    
    % What follows aims at defining the hessian matrix H, the bias
    % vector g, and the constraint matrix A for the formalism of qpOases,ie
    %
    %
    % min (1/2) x'*H*x + x'*g
    % s.t.
    %     lbA < A*x < ubA
    %     lb  < x   < ub
    %
    % For further information, see
    % 
    % http://www.coin-or.org/qpOASES/doc/3.0/manual.pdf
    %
    % 
    
    H   = HessianMatrixQP;
    g   = biasVectorQP;
    A   = [feetAccelerationConstraintMatrix;
           frictionConeConstraintMatrix];
    lbA = [ feetAccelerationConstraintUpperBoundVector;
           -constantMaxTolerance * ones(size(frictionConeUpperBoundVector))];
    ubA = [ feetAccelerationConstraintUpperBoundVector;
            frictionConeUpperBoundVector];
    lb  = [-60   * ones(ROBOT_DOF,1);
           -1000 * ones(12,1)];
    ub  = [ 60   * ones(ROBOT_DOF,1);
            1000 * ones(12,1)];
    
    %Ensure symmetry of H
    H = (H + H')/2;
          
    [u,~,exitFlagQP,~,~,~] = qpOASES(H,g,A,lb,ub,lbA,ubA);     
    
%     if exitFlagQP ~= 0 
%         u = uPrevious;
%     else
%         uPrevious = u;
%     end

%Outputs: 
    %Torques
    block.OutputPort(1).Data = u(1:ROBOT_DOF);
    %Contact forces at the feet (left, right)
    block.OutputPort(2).Data = u(ROBOT_DOF+1:ROBOT_DOF+6);
    block.OutputPort(3).Data = u(ROBOT_DOF+7:ROBOT_DOF+12);
    %QP exit flag
    block.OutputPort(4).Data = exitFlagQP;
    
%end Outputs


function Terminate(~) %Terminate(block)

%end Terminate

function SetOutputPortDimensions(s, port, dimsInfo)
    s.OutputPort(port).Dimensions = dimsInfo;
    
function SetInputPortDimensions(s, port, dimsInfo)
    s.InputPort(port).Dimensions = dimsInfo;


