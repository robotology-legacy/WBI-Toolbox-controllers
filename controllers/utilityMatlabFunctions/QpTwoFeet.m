function QpTwoFeet(block)

setup(block);

function setup(block)
    
block.NumInputPorts  = 6; 
block.NumOutputPorts = 2; 

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;
% block.SetPreCompInpPortInfoToInherited;
% block.SetPreCompOutPortInfoToInherited;

% %1 % [quadTerm;linTerm] (12x12;1x12):(13x12)               
% %2 % [Aineq,bineq]             
% %3 % [Aeq,beq]                   
% %4 % [x0;lb;ub]                  

% for now based on both feet (needs to be defined dynamically)
% Override input port properties
block.InputPort(1).Dimensions        = [ 1  2];   % LEFT_RIGHT_FOOT_IN_CONTACT
block.InputPort(2).Dimensions        = [12 12];   % HessianMatrixQP2Feet               
block.InputPort(3).Dimensions        = [ 1 12];   % gradientQP2Feet
block.InputPort(4).Dimensions        = [38 12];   % ConstraintsMatrixQP2Feet 
block.InputPort(5).Dimensions        = [ 1 38];   % bVectorConstraintsQp2Feet 
block.InputPort(6).Dimensions        = 1 ;        % USE_QPO_SOLVER
% Override output port properties
block.OutputPort(1).Dimensions       = 12;
block.OutputPort(2).Dimensions       = 1;

% Override input and output port properties
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(4).DatatypeID  = 0;  % double
block.InputPort(5).DatatypeID  = 0;  % double
block.InputPort(6).DatatypeID  = 0;  % double

block.InputPort(1).Complexity  = 'Real';
block.InputPort(2).Complexity  = 'Real';
block.InputPort(3).Complexity  = 'Real';
block.InputPort(4).Complexity  = 'Real';
block.InputPort(5).Complexity  = 'Real';
block.InputPort(6).Complexity  = 'Real';

block.InputPort(1).DirectFeedthrough = true;
block.InputPort(2).DirectFeedthrough = true;
block.InputPort(3).DirectFeedthrough = true;
block.InputPort(4).DirectFeedthrough = true;
block.InputPort(5).DirectFeedthrough = true;
block.InputPort(6).DirectFeedthrough = true;

block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

block.OutputPort(2).DatatypeID  = 0; % boolean   %8; % boolean
block.OutputPort(2).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 0;

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

%end setup

function SetInputPortSamplingMode(block, idx, fd)
 block.InputPort(idx).SamplingMode = fd;

block.OutputPort(1).SamplingMode = fd;
block.OutputPort(2).SamplingMode = fd;


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
    
    LEFT_RIGHT_FOOT_IN_CONTACT                    = block.InputPort(1).Data;
    exitFlag                   = 0;
    
    if sum(LEFT_RIGHT_FOOT_IN_CONTACT) > 1.98
        HessianMatrixQP2Feet       = block.InputPort(2).Data;
        gradientQP2Feet            = block.InputPort(3).Data;
        ConstraintsMatrixQP2Feet   = block.InputPort(4).Data;
        bVectorConstraintsQp2Feet  = block.InputPort(5).Data;
        USE_QPO_SOLVER              = block.InputPort(6).Data;
        if USE_QPO_SOLVER == 1
            [desiredf0,~,exitFlag,iter,lambda,auxOutput] = qpOASES(HessianMatrixQP2Feet,gradientQP2Feet',ConstraintsMatrixQP2Feet,[],[],[],bVectorConstraintsQp2Feet');           
            if exitFlag ~= 0
%                 disp('QP failed with');
%                 exitFlag
%                 iter
%                 auxOutput
%                 lambda
%                 desiredf0 = zeros(6*2,1);
%                 exitFlag  = 1;
                desiredf0 = - inv(HessianMatrixQP2Feet)*gradientQP2Feet';
            end
        else
            desiredf0 = - inv(HessianMatrixQP2Feet)*gradientQP2Feet';
        end
            
    else
            desiredf0 = zeros(6*2,1);
    end
    block.OutputPort(1).Data = desiredf0;
    block.OutputPort(2).Data = exitFlag;
%end Outputs


function Terminate(block)

%end Terminate





