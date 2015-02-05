function controllerQP(block)

setup(block);

function setup(block)
    
block.NumInputPorts  = 16; 
block.NumOutputPorts = 1; 

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;
% block.SetPreCompInpPortInfoToInherited;
% block.SetPreCompOutPortInfoToInherited;

% %1 % q                   = block.InputPort(1).Data;
% %2 % qjInit              = block.InputPort(2).Data;
% %3 % v                   = block.InputPort(3).Data;
% %4 % M                   = block.InputPort(4).Data;
% %5 % h                   = block.InputPort(5).Data;
% %6 % g                   = block.InputPort(6).Data;
% %7 % H                   = block.InputPort(7).Data;
% %8 % PosRightFoot        = block.InputPort(8).Data;
% %9 % Jc                  = block.InputPort(9).Data;
% %10% JcDqD               = block.InputPort(10).Data;
% %11% pos_CoM             = block.InputPort(11).Data;
% %12% J_CoM               = block.InputPort(12).Data;
% %13% Desired_x_dx_ddx_CoM= block.InputPort(13).Data;
% %14% Gains               = block.InputPort(14).Data;
% %15% Impedances          = block.InputPort(15).Data;
% %16% IntErrorCoM         = block.InputPort(16).Data;


% Override input port properties
block.InputPort(1).Dimensions        = 25;%
block.InputPort(2).Dimensions        = 25;%
block.InputPort(3).Dimensions        = 31;%
block.InputPort(4).Dimensions        = 31*31;%  
block.InputPort(5).Dimensions        = 31;%
block.InputPort(6).Dimensions        = 31;%
block.InputPort(7).Dimensions        = 6;%
block.InputPort(8).Dimensions        = 7;%
block.InputPort(9).Dimensions        = 12*31;% this later needs to be dynamic
block.InputPort(10).Dimensions       = 12; % this later needs to be dynamic
block.InputPort(11).Dimensions       = 3;%
block.InputPort(12).Dimensions       = 6*31;%
block.InputPort(13).Dimensions       = 9;%
block.InputPort(14).Dimensions       = 4;%
block.InputPort(15).Dimensions       = 25;%
block.InputPort(16).Dimensions       = 3;%

block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(4).DatatypeID  = 0;  % double
block.InputPort(5).DatatypeID  = 0;  % double
block.InputPort(6).DatatypeID  = 0;  % double
block.InputPort(7).DatatypeID  = 0;  % double
block.InputPort(8).DatatypeID  = 0;  % double
block.InputPort(9).DatatypeID  = 0;  % double
block.InputPort(10).DatatypeID  = 0;  % double
block.InputPort(11).DatatypeID  = 0;  % double
block.InputPort(12).DatatypeID  = 0;  % double
block.InputPort(13).DatatypeID  = 0;  % double
block.InputPort(14).DatatypeID  = 0;  % double
block.InputPort(15).DatatypeID  = 0;  % double
block.InputPort(16).DatatypeID  = 0;  % double

block.InputPort(1).Complexity  = 'Real';
block.InputPort(2).Complexity  = 'Real';
block.InputPort(3).Complexity  = 'Real';
block.InputPort(4).Complexity  = 'Real';
block.InputPort(5).Complexity  = 'Real';
block.InputPort(6).Complexity  = 'Real';
block.InputPort(7).Complexity  = 'Real';
block.InputPort(8).Complexity  = 'Real';
block.InputPort(9).Complexity  = 'Real';
block.InputPort(10).Complexity  = 'Real';
block.InputPort(11).Complexity  = 'Real';
block.InputPort(12).Complexity  = 'Real';
block.InputPort(13).Complexity  = 'Real';
block.InputPort(14).Complexity  = 'Real';
block.InputPort(15).Complexity  = 'Real';
block.InputPort(16).Complexity  = 'Real';

block.InputPort(1).DirectFeedthrough = true;
block.InputPort(2).DirectFeedthrough = true;
block.InputPort(3).DirectFeedthrough = true;
block.InputPort(4).DirectFeedthrough = true;
block.InputPort(5).DirectFeedthrough = true;
block.InputPort(6).DirectFeedthrough = true;
block.InputPort(7).DirectFeedthrough = true;
block.InputPort(8).DirectFeedthrough = true;
block.InputPort(9).DirectFeedthrough = true;
block.InputPort(10).DirectFeedthrough = true;
block.InputPort(11).DirectFeedthrough = true;
block.InputPort(12).DirectFeedthrough = true;
block.InputPort(13).DirectFeedthrough = true;
block.InputPort(14).DirectFeedthrough = true;
block.InputPort(15).DirectFeedthrough = true;
block.InputPort(16).DirectFeedthrough = true;


% Override output port properties
block.OutputPort(1).Dimensions       = 25;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

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
block.RegBlockMethod('Outputs', @Outputs);     % Required
% block.RegBlockMethod('Update', @Update);
% block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required    

%end setup

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C-Mex counterpart: mdlSetWorkWidths
%%
% function DoPostPropSetup(block)
% 
% numberOfPoints = 2; %number of points in a quadrant
% block.NumDworks = 1;
%   
%   block.Dwork(1).Name            = 'A';
%   block.Dwork(1).Dimensions      = 2 * 12 * (4 * (numberOfPoints - 2) + 4);
%   block.Dwork(1).DatatypeID      = 0;      % double
%   block.Dwork(1).Complexity      = 'Real'; % real
%   block.Dwork(1).UsedAsDiscState = false;
  

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

% %compute friction cones contraints
% staticFrictionCoefficient = 0.45;
% %approximation with straight lines
% numberOfPoints = 2; %number of points in a quadrant
% 
% %split the pi/2 angle into numberOfPoints - 1;
% segmentAngle = pi/2 / (numberOfPoints - 1);
% 
% %define angle
% angle = 0 : segmentAngle : (2 * pi - segmentAngle);
% points = [cos(angle); sin(angle)];
% numberOfEquations = size(points, 2);
% assert(size(points, 2) == (4 * (numberOfPoints - 2) + 4));
% 
% %A*x <= b, with b is all zeros.
% A = zeros(numberOfEquations, 6);
% 
% %define equations
% for i = 1 : numberOfEquations
%    firstPoint = points(:, i);
%    secondPoint = points(:, rem(i, numberOfEquations) + 1);
%    
%    %define line passing through the above points
%    angularCoefficients = (secondPoint(2) - firstPoint(2)) / (secondPoint(1) - firstPoint(1));
%    offsets = firstPoint(2) - angularCoefficients * firstPoint(1);
% 
%    inequalityFactor = +1;
%    %if any of the two points are between pi and 2pi, then the inequality is
%    %in the form of y >= m*x + q, and I need to change the sign of it.
%    if (angle(i) > pi || angle(rem(i, numberOfEquations) + 1) > pi)
%        inequalityFactor = -1;
%    end
%    
%    %a force is 6 dimensional f = [fx, fy, fz, mux, muy, muz]'
%    %I have constraints on fx and fy, and the offset will be multiplied by
%    %mu * fz
%    
%    A(i,:) = inequalityFactor .* [-angularCoefficients, 1, -offsets * staticFrictionCoefficient, 0, 0, 0];
%    
% end
% 
% %I have to duplicate the matrices and vector for the two feet
% A = [A, zeros(size(A));
%     zeros(size(A)), A];
% 
% %reshape matrix into single vector
% A = reshape(A, 12 * 2 * numberOfEquations, 1);
% block.Dwork(1).Data = A;


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

% function [tau,CoMError]  =  controllerQP(q,qjInit,v, M, h, g, H, PosRightFoot, Jc, JcDqD, pos_CoM,J_CoM, Desired_x_dx_ddx_CoM, Gains, Impedances, IntErrorCoM)

q                   = block.InputPort(1).Data;
qjInit              = block.InputPort(2).Data;
v                   = block.InputPort(3).Data;
temp                = block.InputPort(4).Data;
M                   = reshape(temp,31,31);
h                   = block.InputPort(5).Data;
g                   = block.InputPort(6).Data;
H                   = block.InputPort(7).Data;
PosRightFoot        = block.InputPort(8).Data;
temp                = block.InputPort(9).Data;
Jc                  = reshape(temp,12,31);
JcDqD               = block.InputPort(10).Data;
pos_CoM             = block.InputPort(11).Data;
temp                = block.InputPort(12).Data;
J_CoM               = reshape(temp,6,31);
temp                = block.InputPort(13).Data;
Desired_x_dx_ddx_CoM= reshape(temp,3,3);
Gains               = block.InputPort(14).Data;
Impedances          = block.InputPort(15).Data;
IntErrorCoM         = block.InputPort(16).Data;


% 
% block.InputPort(1).Dimensions        = 25;%
% block.InputPort(2).Dimensions        = 25;%
% block.InputPort(3).Dimensions        = 31;%
% block.InputPort(4).Dimensions        = 31*31;%  
% block.InputPort(5).Dimensions        = 31;%
% block.InputPort(6).Dimensions        = 31;%
% block.InputPort(7).Dimensions        = 6;%
% block.InputPort(8).Dimensions        = 7;%
% block.InputPort(9).Dimensions        = 12*31;% this later needs to be dynamic
% block.InputPort(10).Dimensions       = 12; % this later needs to be dynamic
% block.InputPort(11).Dimensions       = 3;%
% block.InputPort(12).Dimensions       = 6*31;%
% block.InputPort(13).Dimensions       = 3*3;%
% block.InputPort(14).Dimensions       = 4;%
% block.InputPort(15).Dimensions       = 25;%
% block.InputPort(16).Dimensions       = 3;%

n_dof = 25;
n_constraint=2;

PINV_TOL                = 1e-5;
gravAcc                 = 9.81;

m= M(1,1);
St    = [zeros(6,n_dof);eye(n_dof,n_dof)];
% grav = [-m*gravAcc;zeros(2,1);zeros(3,1)];
grav = [zeros(2,1);-m*gravAcc;zeros(3,1)];

pos_rightFoot = PosRightFoot(1:3);
JcMinv        = Jc/M;
JcMinvSt = JcMinv*St;
JcMinvJct = JcMinv*transpose(Jc);

xDcom      = J_CoM(1:3,:)*v;
xDDcomStar = Desired_x_dx_ddx_CoM(:,3) - Gains(1)*(pos_CoM - Desired_x_dx_ddx_CoM(:,1)) - Gains(2)*IntErrorCoM - Gains(3)*(xDcom - Desired_x_dx_ddx_CoM(:,2));
clc
Desired_x_dx_ddx_CoM(:,1)

Pr =  pos_rightFoot - pos_CoM; % Application point of the contact force on the right foot w.r.t. CoM

switch n_constraint
    case 1 % on left foot
        A  = [ eye(3),   zeros(3);
             -Sf(pos_CoM),  eye(3)];
    case 2 % on both feet
        A  = [ eye(3),   zeros(3),eye(3), zeros(3);
             -Sf(pos_CoM),  eye(3), Sf(Pr), eye(3) ];
    otherwise
        disp('Choose number of constraints properly (1 or 2)');
        return
end        

pinvA         = pinv(        A, PINV_TOL);
PInv_JcMinvSt = pinv( JcMinvSt, PINV_TOL);
HDotDes       = [ m*xDDcomStar ;
                -Gains(4)*H(4:end)]; 
         
% QP FOR TORQUE MINIMIZATION BASED ON arbitrary vector F0

options = optimset('Algorithm','active-set','Display','off');

%define tau_0 (the arbitrary vector)
mult_f_tau0     = -transpose(Jc(:,7:end)); % multiplier of f in tau0
n_tau0          = g(7:end) - (Impedances.*(q-qjInit)); % additional terms in tau0
%

N0_tau          = eye(n_dof) - PInv_JcMinvSt*JcMinvSt;
mult_f_tau      = PInv_JcMinvSt*(-JcMinvJct)+N0_tau*mult_f_tau0;
n_tau           = PInv_JcMinvSt*(JcMinv*h-JcDqD)+N0_tau*n_tau0;

% f desired is given as:
% f*   = pinvA*(HDotDes - grav)  + (I-pinvA*A) * f0
% f*   =           c_f           +     N0_f    * f0
N0_f            = eye(6*n_constraint,6*n_constraint)-pinvA*A;
c_f             = pinvA*(HDotDes - grav);

% tau is given as:
% tau = (mult_f_tau*N0_f)* f0 + (n_tau+mult_f_tau*c_f)
% tau =        X_tau     * f0 +                   Y_tau

X_tau = mult_f_tau*N0_f;
Y_tau = n_tau+mult_f_tau*c_f;
quadraticTerm2 = 2*(transpose(X_tau)*X_tau)+eye(6*n_constraint)*1e-8;
linearTerm2 = transpose(2*X_tau'*Y_tau);

% CONSTRAINTS

    %inequalities
    
        %friction
staticFrictionCoefficient = 2;
numberOfPoints = 4; %number of points in a quadrant for cone

% [Aineq_fcone_f,bineq_fcone_f] = constraint_fcone(staticFrictionCoefficient,numberOfPoints,n_constraint);

%  function [Aineq,bineq]=constraint_fcone(staticFrictionCoefficient,numberOfPoints,n_constraint)
%compute friction cones contraints

%approximation with straight lines

%split the pi/2 angle into numberOfPoints - 1;
segmentAngle = pi/2 / (numberOfPoints - 1);

%define angle
angle = 0 : segmentAngle : (2 * pi - segmentAngle);
points = [cos(angle); sin(angle)];
numberOfEquations = size(points, 2);
assert(size(points, 2) == (4 * (numberOfPoints - 2) + 4));

%Aineq*x <= b, with b is all zeros.
Aineqtemp = zeros(numberOfEquations, 6);

%define equations
for i = 1 : numberOfEquations
   firstPoint = points(:, i);
   secondPoint = points(:, rem(i, numberOfEquations) + 1);
   
   %define line passing through the above points
   angularCoefficients = (secondPoint(2) - firstPoint(2)) / (secondPoint(1) - firstPoint(1));
   
   offsets = firstPoint(2) - angularCoefficients * firstPoint(1);

   inequalityFactor = +1;
   %if any of the two points are between pi and 2pi, then the inequality is
   %in the form of y >= m*x + q, and I need to change the sign of it.
   if (angle(i) > pi || angle(rem(i, numberOfEquations) + 1) > pi)
       inequalityFactor = -1;
   end
   
   %a force is 6 dimensional f = [fx, fy, fz, mux, muy, muz]'
   %I have constraints on fy and fz, and the offset will be multiplied by
   %mu * fx
   
   Aineqtemp(i,:) = inequalityFactor .* [-offsets * staticFrictionCoefficient,-angularCoefficients, 1, 0, 0, 0];  
   
end

switch n_constraint
    case 1
        bineq_fcone_f = zeros(size(Aineqtemp,1), 1);
    case 2    
        %duplicate the matrices and vector for the two feet
        Aineq_fcone_f = [Aineqtemp, zeros(size(Aineqtemp));
                zeros(size(Aineqtemp)), Aineqtemp];
          
        bineq_fcone_f = zeros(size(Aineq_fcone_f,1), 1);
    otherwise
end
%end friction cone 

Aineq = Aineq_fcone_f*N0_f;
bineq = bineq_fcone_f-Aineq_fcone_f*c_f;
    
        %moments
        
%add constraints on moments

% Aineq_moment_f = [0, 0, 0, 0, 1, 0,zeros(1,6);
%         0, 0, 0, 0,-1, 0,zeros(1,6);
%         0, 0, 0, 0, 0, 1,zeros(1,6);
%         0, 0, 0, 0, 0,-1,zeros(1,6);
%         zeros(1,6), 0, 0, 0, 0, 1, 0;
%         zeros(1,6), 0, 0, 0, 0,-1, 0;
%         zeros(1,6), 0, 0, 0, 0, 0, 1;
%         zeros(1,6), 0, 0, 0, 0, 0,-1];
% bineq_moment_f = 2*[5;5;5;5;5;5;5;5];
% 
% Aineq_moment_f0 = Aineq_moment_f*N0_f;
% bineq_moment_f0 = bineq_moment_f-Aineq_moment_f*c_f;
% 
% Aineq = [Aineq;Aineq_moment_f0];
% bineq = [bineq;bineq_moment_f0];
        





% Aeq = JcMinv*St*X_tau + JcMinvJct*(eye(12,12)-pinvA*A);
% beq = JcMinv*h - JcDqD - JcMinv*St*Y_tau - JcMinvJct*c_f;


% INITIAL CONDITION
x0 = zeros(6*n_constraint,1);

% BOUNDS ON F0
lb = [];
ub = [];

% [desiredf0, objVal, exitFlag, output, lambda] = ... 
[desiredf0, ~, exitFlag, ~, ~] = ...
quadprog(quadraticTerm2, linearTerm2, ...
          [], [], ... %inequalities
          [], [], ... %equalities
          lb, ub, ... %bounds
          x0,     ... %initial solution
          options);

if exitFlag~=1
    disp('qp_tau_f0 failed');
    desiredf0 = x0;
end
% % output
% % [x0 , desiredFeetContactForces]

% the old controller
desiredFeetContactForces = pinvA*(HDotDes-grav); % to ignore QP

measuredFeetContactForces = desiredFeetContactForces;

tauForDesiredFeetForces = PInv_JcMinvSt*(JcMinv*h - JcDqD - JcMinvJct*measuredFeetContactForces);
tauForImpedenceBehaviour = g(7:end)-Jc(:,7:end)'*measuredFeetContactForces - (Impedances.*(q-qjInit));

tau_old = tauForDesiredFeetForces + N0_tau*tauForImpedenceBehaviour;

tau = X_tau*desiredf0+Y_tau;
% tau = zeros(25,1);
% [tau,tau_old]
CoMError    = pos_CoM - Desired_x_dx_ddx_CoM(:,1);

% normCoMError  = norm(CoMError);

block.OutputPort(1).Data = tau;
% block.OutputPort(2).Data = CoMError;

%end Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlUpdate
%%
% function Update(block)
% 
% block.Dwork(1).Data = block.InputPort(1).Data;

%end Update

%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlDerivatives
%%
% function Derivatives(block)

%end Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C-MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate





% function [tau,CoMError]  =  controllerQP(q,qjInit,v, M, h, g, H, PosRightFoot, Jc, JcDqD, pos_CoM,J_CoM, Desired_x_dx_ddx_CoM, Gains, Impedances, IntErrorCoM)
% 
% % t...
%     
% 
% n_dof = 25;
% n_constraint=2;
% 
% PINV_TOL                = 1e-5;
% gravAcc                 = 9.81;
% 
% m= M(1,1);
% St    = [zeros(6,n_dof);eye(n_dof,n_dof)];
% grav = [-m*gravAcc;zeros(2,1);zeros(3,1)];
% pos_rightFoot = PosRightFoot(1:3);
% JcMinv        = Jc/M;
% JcMinvSt = JcMinv*St;
% JcMinvJct = JcMinv*transpose(Jc);
% 
% % to_store = [prm.joint_limits(:,1),q,prm.joint_limits(:,2)];
% % joints_vs_lim = (to_store(:,2)-to_store(:,1))./(to_store(:,3)-to_store(:,1));
% % joints_above_limits = joints_vs_lim>=1 | joints_vs_lim<=0;
% 
% % if isempty(to_store(joints_above_limits))
% % else
% % %     clc
% % %     disp('joint limits reached!');
% % %     [to_store , joints_above_limits]
% %     
% % end
% % [to_store , joints_above_limits]
% 
% 
% xDcom      = J_CoM(1:3,:)*v;
% xDDcomStar = Desired_x_dx_ddx_CoM(:,3) - Gains(1)*(pos_CoM - Desired_x_dx_ddx_CoM(:,1)) - Gains(2)*IntErrorCoM - Gains(3)*(xDcom - Desired_x_dx_ddx_CoM(:,2));
% 
% Pr =  pos_rightFoot - pos_CoM; % Application point of the contact force on the right foot w.r.t. CoM
% 
% switch n_constraint
%     case 1 % on left foot
%         A  = [ eye(3),   zeros(3);
%              -Sf(pos_CoM),  eye(3)];
%     case 2 % on both feet
%         A  = [ eye(3),   zeros(3),eye(3), zeros(3);
%              -Sf(pos_CoM),  eye(3), Sf(Pr), eye(3) ];
%     otherwise
%         disp('Choose number of constraints properly (1 or 2)');
%         return
% end        
% 
% pinvA         = pinv(        A, PINV_TOL);
% PInv_JcMinvSt = pinv( JcMinvSt, PINV_TOL);
% HDotDes       = [ m*xDDcomStar ;
%                 -Gains(4)*H(4:end)]; 
%          
% %% QP FOR TORQUE MINIMIZATION BASED ON arbitrary vector F0
% 
% % options = optimset('Algorithm','active-set','Display','off');
% 
% %define tau_0 (the arbitrary vector)
% mult_f_tau0     = -transpose(Jc(:,7:end)); % multiplier of f in tau0
% n_tau0          = g(7:end) - (Impedances.*(q-qjInit)); % additional terms in tau0
% %
% 
% N0_tau          = eye(n_dof) - PInv_JcMinvSt*JcMinvSt;
% mult_f_tau      = PInv_JcMinvSt*(-JcMinvJct)+N0_tau*mult_f_tau0;
% n_tau           = PInv_JcMinvSt*(JcMinv*h-JcDqD)+N0_tau*n_tau0;
% 
% % f desired is given as:
% % f*   = pinvA*(HDotDes - grav)  + (I-pinvA*A) * f0
% % f*   =           c_f           +     N0_f    * f0
% N0_f            = eye(6*n_constraint,6*n_constraint)-pinvA*A;
% c_f             = pinvA*(HDotDes - grav);
% 
% % tau is given as:
% % tau = (mult_f_tau*N0_f)* f0 + (n_tau+mult_f_tau*c_f)
% % tau =        X_tau     * f0 +                   Y_tau
% 
% X_tau = mult_f_tau*N0_f;
% Y_tau = n_tau+mult_f_tau*c_f;
% quadraticTerm2 = 2*(transpose(X_tau)*X_tau)+eye(6*n_constraint)*1e-8;
% linearTerm2 = transpose(2*X_tau'*Y_tau);
% 
% % CONSTRAINTS
% 
%     %inequalities
%     
%         %friction
% staticFrictionCoefficient = 2;
% numberOfPoints = 4; %number of points in a quadrant for cone
% 
% [Aineq_fcone_f,bineq_fcone_f] = constraint_fcone(staticFrictionCoefficient,numberOfPoints,n_constraint);
% 
% Aineq = Aineq_fcone_f*N0_f;
% bineq = bineq_fcone_f-Aineq_fcone_f*c_f;
%     
%         %moments
%         
% %add constraints on moments
% 
% % Aineq_moment_f = [0, 0, 0, 0, 1, 0,zeros(1,6);
% %         0, 0, 0, 0,-1, 0,zeros(1,6);
% %         0, 0, 0, 0, 0, 1,zeros(1,6);
% %         0, 0, 0, 0, 0,-1,zeros(1,6);
% %         zeros(1,6), 0, 0, 0, 0, 1, 0;
% %         zeros(1,6), 0, 0, 0, 0,-1, 0;
% %         zeros(1,6), 0, 0, 0, 0, 0, 1;
% %         zeros(1,6), 0, 0, 0, 0, 0,-1];
% % bineq_moment_f = 2*[5;5;5;5;5;5;5;5];
% % 
% % Aineq_moment_f0 = Aineq_moment_f*N0_f;
% % bineq_moment_f0 = bineq_moment_f-Aineq_moment_f*c_f;
% % 
% % Aineq = [Aineq;Aineq_moment_f0];
% % bineq = [bineq;bineq_moment_f0];
%         
% 
% 
% 
% 
% 
% % Aeq = JcMinv*St*X_tau + JcMinvJct*(eye(12,12)-pinvA*A);
% % beq = JcMinv*h - JcDqD - JcMinv*St*Y_tau - JcMinvJct*c_f;
% 
% 
% % INITIAL CONDITION
% x0 = zeros(6*n_constraint,1);
% 
% % BOUNDS ON F0
% lb = [];
% ub = [];
% 
% % [desiredf0, objVal, exitFlag, output, lambda] = ... 
% [desiredf0, ~, ~, ~, ~] = ...
% quadprog(quadraticTerm2, linearTerm2, ...
%           Aineq, bineq, ... %inequalities
%           [], [], ... %equalities
%           lb, ub, ... %bounds
%           x0,     ... %initial solution
%           'Algorithm','active-set','Display','off');
% %           options);
% % clc
% % % objVal+0.5*(grav-HDotDes)'*(grav-HDotDes)
% % exitFlag
% if exitFlag~=1
%     disp('qp_tau_f0 failed');
% end
% % % output
% % % [x0 , desiredFeetContactForces]
% 
% 
% 
% 
% 
% 
% %%
% % desiredFeetContactForces = pinvA*(HDotDes-grav); % to ignore QP
% % 
% % measuredFeetContactForces = desiredFeetContactForces;
% % 
% % tauForDesiredFeetForces = PInv_JcMinvSt*(JcMinv*h - JcDqD - JcMinvJct*measuredFeetContactForces);
% % tauForImpedenceBehaviour = g(7:end)-Jc(:,7:end)'*measuredFeetContactForces - (Impedances.*(q-qjInit));
% % 
% % tau = tauForDesiredFeetForces + N0*tauForImpedenceBehaviour;
% 
% tau = X_tau*desiredf0+Y_tau;
% 
% CoMError    = pos_CoM - Desired_x_dx_ddx_CoM(:,1);
% 
% 
% normCoMError  = norm(CoMError);
% end




% % function [Aineq,bineq]=constraint_fcone(staticFrictionCoefficient,numberOfPoints,n_constraint)
% % %compute friction cones contraints
% % 
% % %approximation with straight lines
% % 
% % %split the pi/2 angle into numberOfPoints - 1;
% % segmentAngle = pi/2 / (numberOfPoints - 1);
% % 
% % %define angle
% % angle = 0 : segmentAngle : (2 * pi - segmentAngle);
% % points = [cos(angle); sin(angle)];
% % numberOfEquations = size(points, 2);
% % assert(size(points, 2) == (4 * (numberOfPoints - 2) + 4));
% % 
% % %Aineq*x <= b, with b is all zeros.
% % Aineqtemp = zeros(numberOfEquations, 6);
% % 
% % %define equations
% % for i = 1 : numberOfEquations
% %    firstPoint = points(:, i);
% %    secondPoint = points(:, rem(i, numberOfEquations) + 1);
% %    
% %    %define line passing through the above points
% %    angularCoefficients = (secondPoint(2) - firstPoint(2)) / (secondPoint(1) - firstPoint(1));
% %    
% %    offsets = firstPoint(2) - angularCoefficients * firstPoint(1);
% % 
% %    inequalityFactor = +1;
% %    %if any of the two points are between pi and 2pi, then the inequality is
% %    %in the form of y >= m*x + q, and I need to change the sign of it.
% %    if (angle(i) > pi || angle(rem(i, numberOfEquations) + 1) > pi)
% %        inequalityFactor = -1;
% %    end
% %    
% %    %a force is 6 dimensional f = [fx, fy, fz, mux, muy, muz]'
% %    %I have constraints on fy and fz, and the offset will be multiplied by
% %    %mu * fx
% %    
% %    Aineqtemp(i,:) = inequalityFactor .* [-offsets * staticFrictionCoefficient,-angularCoefficients, 1, 0, 0, 0];  
% %    
% % end
% % 
% % switch n_constraint
% %     case 1
% %         bineq = zeros(size(Aineqtemp,1), 1);
% %     case 2    
% %         %duplicate the matrices and vector for the two feet
% %         Aineq = [Aineqtemp, zeros(size(Aineqtemp));
% %                 zeros(size(Aineqtemp)), Aineqtemp];
% %           
% %         bineq = zeros(size(Aineq,1), 1);
% %     otherwise
% % end
% % 
% % end

% % % 
% % % PINV_TOL = 1e-5;
% % % gravAcc  = 9.81;
% % % 
% % % n = 25;
% % % St    = [zeros(6,n);eye(n,n)];
% % % 
% % % m     = M(1,1);
% % % Mb    = M(1:6,1:6);
% % % Mbj   = M(1:6,7:end);
% % % 
% % % grav = [zeros(2,1);-m*gravAcc;zeros(3,1)];
% % % 
% % % % Definition of reference accelerations for CoM and Joints that ensure
% % % % stabilization of desired trajectories xDcomDes and qDes 
% % % 
% % % xDcom      = Jcom(1:3,:)*qD;
% % % xDDcomStar = Desired_x_dx_ddx_CoM(:,3) - Gains(1)*(xcom - Desired_x_dx_ddx_CoM(:,1)) - Gains(2)*IntErrorCoM - Gains(3)*(xDcom - Desired_x_dx_ddx_CoM(:,2));
% % % 
% % % Pr =  PosRightFoot(1:3) - xcom; % Application point of the contact force on the right foot w.r.t. CoM
% % % 
% % % A  = [ eye(3),   zeros(3),eye(3), zeros(3);
% % %       -Sf(xcom),  eye(3), Sf(Pr), eye(3) ];
% % % pinvA = pinv(A, PINV_TOL);
% % %   
% % % HDotDes  = [ m*xDDcomStar ;
% % %             -Gains(4)*H(4:end)]; 
% % % 
% % % Jc    =   feetJacobians;
% % %            
% % % JcDqD = feetJdqd;
% % %             
% % % desiredFeetContactForces = pinvA*(HDotDes-grav);    
% % % 
% % % JcMinv        = Jc/M;
% % % PInv_JcMinvSt = pinv(JcMinv*St, PINV_TOL);
% % % N0            = eye(n) - PInv_JcMinvSt*JcMinv*St;
% % % 
% % % measuredFeetContactForces = desiredFeetContactForces;
% % % 
% % % tauForDesiredFeetForces = PInv_JcMinvSt*(JcMinv*h - JcDqD - JcMinv*Jc'*measuredFeetContactForces);
% % % 
% % % tauForImpedenceBehaviour = - Impedances.*(q-qInit) + g(7:end)-Jc(:,7:end)'*measuredFeetContactForces - Mbj'/Mb*(h(1:6) - Jc(:,1:6)'*measuredFeetContactForces);
% % % 
% % % tau = tauForDesiredFeetForces + N0*tauForImpedenceBehaviour;
% % % 
% % % CoMError    = xcom - Desired_x_dx_ddx_CoM(:,1);
% % % % normCoMError  = norm(CoMError);
% % % 
% % % 
