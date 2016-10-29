%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci
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

function QpBalancingSOT(block)

setup(block);

function setup(block)
    
block.NumInputPorts  = 8; 
block.NumOutputPorts = 4; 

% Setup port properties to be  dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;


% Definition of port sizes for QP 2 feet
%block.InputPort(1).Dimensions        = -1;  It does not copile if the
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
block.NumDialogPrms     = 1;

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


    CONTACT_THRESHOLD = 0.1;
    
    LEFT_RIGHT_FOOT_IN_CONTACT = block.InputPort(1).Data;
    exitFlagQP                 = 0;
    HessianMatrixQP2Feet       = block.InputPort(2).Data;
    gradientQP2Feet            = block.InputPort(3).Data;
    ConstraintsMatrixQP2Feet   = block.InputPort(4).Data;
    bVectorConstraintsQp2Feet  = block.InputPort(5).Data;
    if sum(LEFT_RIGHT_FOOT_IN_CONTACT) > (2 - CONTACT_THRESHOLD)
        if USE_QPO_SOLVER 
            [f02Feet,~,exitFlagQP,~,~,~] = qpOASES(HessianMatrixQP2Feet,gradientQP2Feet',ConstraintsMatrixQP2Feet,[],[],[],bVectorConstraintsQp2Feet');           
            if exitFlagQP ~= 0
                f02Feet = - inv(HessianMatrixQP2Feet)*gradientQP2Feet';
            end
        else
            exitFlagQP                 = 1;
            f02Feet = - inv(HessianMatrixQP2Feet)*gradientQP2Feet';
        end
            
    elseif sum(LEFT_RIGHT_FOOT_IN_CONTACT) > (1 - CONTACT_THRESHOLD) 

        if USE_QPO_SOLVER 
            [f0OneFoot,~,exitFlagQP,~,~,~] = qpOASES(HessianMatrixQP1Foot,gradientQP1Foot',ConstraintsMatrixQP1Foot,[],[],[],bVectorConstraintsQP1Foot');           

           if exitFlagQP ~= 0
                f0OneFoot = - inv(HessianMatrixQP1Foot)*gradientQP1Foot';
           end
        else
            exitFlagQP= 1;
            f0OneFoot = - inv(HessianMatrixQP1Foot)*gradientQP1Foot';
        end
    else
        exitFlagQP           = -10;
    end
    block.OutputPort(1).Data = zeros(block.DialogPrm(1).Data,0);
    block.OutputPort(2).Data = zeros(6,0);
    block.OutputPort(3).Data = zeros(6,0);
    block.OutputPort(4).Data = exitFlagQP;
    
%end Outputs


function Terminate(block)


%end Terminate

function SetOutputPortDimensions(s, port, dimsInfo)
    s.OutputPort(port).Dimensions = dimsInfo;
    
function SetInputPortDimensions(s, port, dimsInfo)
    s.InputPort(port).Dimensions = dimsInfo;


