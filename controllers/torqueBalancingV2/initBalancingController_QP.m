%WBC ASW
clear all

% Controller period
Ts             = 0.01; 
simulationTime = inf;
 
% Controller gains for convergence of the desired centroidal momentum. 
% The first three elements are the Proportional, Intagral, and the Derivative
% gains taking place in xComDDStart, i.e. 
%
% xComDDStart = xDDcomDes - Gains(1)*(xcom - xcomDes) - Gains(2)*IntErrorCoM - Gains(3)*(xDcom - xDcomDes)  
%
% The fourth element is the gain for the 
% angular part of the centroidal momentum convergence, i.e. 
%
% hwDot = -Gains(4)*hw  

gains  = [    120    0   1   1 ];

% Impadances acting in the null space of the desired contact forces 


impTorso            = [  70    20   10
                          0     0    0]; 
impArms             = [ 8    8    8   8   8
                        0    0    0   0   0];
impLegs             = [35   70    0    100    100  10
                        0    0   0     1000   1000  0]; 
impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLegs(1,:),impLegs(1,:)];
increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLegs(2,:),impLegs(2,:)];
impedencesSat       = [100   100    1000];


              
% Rotation of the gazebo FT sensor
R   = [0 0 1; 0 -1 0;1 0 0];
Rf  = [R, zeros(3,3); zeros(3,3), R];


referenceParams = [0.0 0.0];  %[0.015 0.5];


%% Parameters for QP
n_constraint = 2;
x0 = zeros(6*n_constraint,1);
lb = -inf*ones(6*n_constraint,1);
ub = inf*ones(6*n_constraint,1);
x0_lb_ub    = [x0;lb;ub];

%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)

n_constraint                 = 2;
% friction cone
staticFrictionCoefficient    = 1/3;  
numberOfPoints               = 4; %number of points in a quadrant for cone
torsionalFrictionCoefficient = 2/150;

[Aineq_fcone_f,bineq_fcone_f]= constraint_fcone_QP(staticFrictionCoefficient,numberOfPoints,n_constraint);


if n_constraint==2
    Aineq_torsion_f = [0 , 0, -torsionalFrictionCoefficient, 0, 0, 1, zeros(1,6);
                       0 , 0, -torsionalFrictionCoefficient, 0, 0,-1, zeros(1,6);
                       zeros(1,6),0 , 0, -torsionalFrictionCoefficient, 0, 0, 1;
                       zeros(1,6),0 , 0, -torsionalFrictionCoefficient, 0, 0,-1];   
    bineq_torsion_f = zeros(4,1);
else
    Aineq_torsion_f = [0 , 0, -torsionalFrictionCoefficient, 0, 0, 1;
                       0 , 0, -torsionalFrictionCoefficient, 0, 0,-1];
    bineq_torsion_f = zeros(2,1); 
end    
%

% merge the constraints together
% here still in terms of f (not f0) 
% (the constraint on f0 are variable and calculated in the controller)
Aineq_f = [Aineq_fcone_f;
           Aineq_torsion_f];
bineq_f = [bineq_fcone_f;
           bineq_torsion_f];

%% GAINS FOR Gazebo



% kCom  = [  40    0.2   0 ];
% kImpTorso = [3 2 4]*8; 
% kImpArms  = [2 2 2 2 1]*8;
% kImpLegs  = [35 10 0.1 40 2 0.5]; 
% 
% Gains  = [  70    0   0  4];
% 
% % Impadances acting in the null space of the desired contact forces 
% 
% kImpTorso = [12 8 12]; 
% kImpArms  = [10 10 10 10 5];
% kImpLegs  = [35 50 0.1 30 2 10]; 

%% GAINS FOR iCubGenova01
% kw = 4;
% kCom  = [  40    0.2   0 ];
% kImpTorso = [3 2 4]*8; 
% kImpArms  = [2 2 2 2 1]*8;
% kImpLegs  = [35 10 0.1 40 2 0.5]; 
% 

%% Right left 
% A = 0.02;
% f = 0.15; up to 0.17
% kH  = [  70    2   0 4];
% kImpTorso = [3 2 3]*4; 
% kImpArms  = [2 2 2 2 1]*5;
% kImpLegs  = [35 50 0.1 30 2 10];  

%% GAINS FOR iCubGenova03
%
% Demo: right left
%
% kCom  = [  40    1   0 ];
% kImpTorso = [1 1 4]*15; 
% kImpArms  = [2 2 2 2 1]*15;
% kImpLegs  = [200 150 250 500 10 1]; 
% kImp  = [kImpTorso,kImpArms,kImpArms,kImpLegs,kImpLegs];
%
% 
% Demo: Constanst CoM
%
% kCom  = [  40    0   0 ];
% kImpTorso = [1 1 2]*15; 
% kImpArms  = [1 1 1 1 1]*15;
% kImpLegs  = [200 150 250 450 10 1]; 