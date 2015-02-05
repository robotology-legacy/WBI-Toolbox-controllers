clear all
robotName = 'icubGazeboSim';
localName = 'simulinkJorh';
Ts        = 0.01;

% Controller period
Ts = 0.01; 
 
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

gainsPCOM                 = diag([80  80 80]);
minCoMx_y                 = [-0.1   -0.25 ];  
maxCoMx_y                 = [ 0.1    0.05 ];
satGainsPCOM              = 300;
increasingRatesGainsPCOM  = [ 0     0    ];

gainsIDMomentum           = [ 0     1   1 ];

% Impadances acting in the null space of the desired contact forces 

impTorso            = [  50    15   10
                          0     0   0]; 
impArms             = [ 8    8    8   8   8
                        0    0    0   0   0];
impLegs             = [35   70    0    1000    1000  10
                        0    0   0       0        0  0]; 

impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLegs(1,:),impLegs(1,:)];

increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLegs(2,:),impLegs(2,:)];

impedencesSat       = [100   100    2000];


dampTorso            = [ 0.1  0.1  0.1]; 
dampArms             = [ 0.1  0.1  0.1 0.1  0.1];
dampLegs             = [ 0.1  0.1  0.1 0.1  0.1 0.1]; 

damping          = [dampTorso,dampArms,dampArms,dampLegs,dampLegs]*0;
              

referenceParams = [0.0 0.0];  %[0.015 0.5];

% Rotation of the gazebo FT sensor
R   = [0 0 1; 0 -1 0;1 0 0];
Rf  = [R, zeros(3,3); zeros(3,3), R];

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
% Gains  = [  120    0   5  4];
% 
% % Impadances acting in the null space of the desired contact forces 
% 
% impTorso            = [12    8   12
%                         0    0    0]; 
% impArms             = [10   10   10  10   5
%                         0    0    0   0   0];
% impLegs             = [35   50    0  30   2  10
%                         0    0    0   0   0   0];  
% 
% Gains  = [  110    0   1  1];
% 
% % Impadances acting in the null space of the desired contact forces 
% 
% impTorso            = [30    30   10
%                         0     0   0]; 
% impArms             = [ 8    8    8   8   8
%                         0    0    0   0   0];
% impLegs             = [35   70    0    1000    1000  10
%                        300  300   0   20000    1000  25]; 
% 
% impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLegs(1,:),impLegs(1,:)];
% 
% increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLegs(2,:),impLegs(2,:)];
% 
% impedencesSat       = [100   100    2000];
%               
% ReferenceParams = [0.015 0.4];
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
