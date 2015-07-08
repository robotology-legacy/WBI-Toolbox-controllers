% ROBOT_DOFS = 25;
% clc;
robotName        = 'icub';
localName        = 'seesawBalancingController';
Ts = 0.01;
seesawKind       = 2;

orientationOffset = [-1.2; 0; 0];

seesaw_inertial  = '/quaternionEKFModule/filteredOrientationEuler:o';
% wbm_modelInitialise(robotName);
% [qj,~,~,~]       = wholeBodyModel('get-state');

ROBOT_DOF        = 23;
addpath(genpath('../matlab'));

model.robot.dofs = ROBOT_DOF;
model.robot.lFootCentreDistance =  0.11;
model.robot.rFootCentreDistance = -0.11;
    
seesaw           = struct;
seesaw.h         = 0.1;
seesaw.rho       = 0.362;%0.175;%0.362; % init value 0.362
seesaw.delta     = seesaw.rho - seesaw.h + 0.002;
seesaw.inertia   = diag([7.6698599e-02, 3.7876787e-02, 1.0893139e-01]);
seesaw.mass      = 4.2;
seesaw.top       = 0.002;% seesaw.delta - (seesaw.rho - seesaw.h) ;
seesaw.kind      = seesawKind;
switch seesaw.kind
    case 1 %Spherical seesaw
        seesaw.iota      = seesaw.mass*inv(seesaw.inertia);
        seesaw.invIota   = inv(seesaw.iota);
    case 2 %Semi cylidrical seesaw
        seesaw.iota      = [1;0;0]*[1;0;0]'*seesaw.mass*inv(seesaw.inertia);
        seesaw.invIota   = 0;
end

regs             = struct;
regs.pinvTol     = 1e-5;
regs.pinvDamp    = 1e-1;
regs.pinvDampA   = 1e-2;


model.seesaw     = seesaw;
%    
% INERTIA TENSOR:
% Ixx Ixy Ixz 7.6698599e-02 0.0000000e+00 0.0000000e+00
% Iyx Iyy Iyz 0.0000000e+00 3.7876787e-02 0.0000000e+00
% Izx Izy Izz 0.0000000e+00 0.0000000e+00 1.0893139e-01

%%Assumptions:
% 1) feet do not move with respect to the seesaw
% 2) feet are centered with the center of mass of the seesaw
% these assumptions are enforced (?) in the function as persistent
% variables

directionOfOscillation            = [0;1;0];
referenceParams                   = [0.00  0.1];  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz

noOscillationTime        = 0; % If DEMO_LEFT_AND_RIGHT = 1, the variable noOscillationTime is the time, in seconds, 
                               % that the robot waits before starting the left-and-righ

%%Gains and references
gains.posturalProp  = diag([10   10    10 ...
                             12   12   12      12      ...
                             12   12   12      12     ...
                            35    0    30      35     55   0,...
                            35    0    30      35     55   0]);
gains.posturalDamp  = diag([ 1    1     1 ...
                             1    1     1       1       ...
                             1    1     1       1       ...
                             1    1     1       1      1   1,...
                             1    1     1       1      1   1])*0;
TorqueMax          = 24;
gains.thetaGain    = 100;
gains.omegaGain    = 10;
gains.xcomPGain    = diag([2.5, 30, 50]);%diag([2.5, 5, 100])%diag([2.5, 5, 5]);
gains.xcomDGain    = 2*sqrt(gains.xcomPGain)*0;
gains.Hw           = 1;
gains.onSeesaw     = 1;
gains.ssawControl  = 0;
