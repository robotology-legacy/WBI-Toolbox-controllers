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


ROBOT_DOF = 23;
gainsPCOM                 = diag([ 40   40  40])/10;
gainsICOM                 = diag([  0    0   0]);
gainsDCOM                 = 2*sqrt(gainsPCOM);

minCoMx_y                 = [-0.1   -0.25 ];  
maxCoMx_y                 = [ 0.1    0.05 ];
satGainsPCOM              = 300;
increasingRatesGainsPCOM  = [ 0     0    ];

gainMomentum           = 1 ;

% Impadances acting in the null space of the desired contact forces 


% 
impTorso            = [   60    20   10
                           0     0    0]; 
impArms             = [ 8    8    8  12   
                        0    0    0   0   ];
impLegs             = [ 35   10    0      350    350  10
                         0    0   0        0      0   0]; 
impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLegs(1,:),impLegs(1,:)];
increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLegs(2,:),impLegs(2,:)];
impedencesSat       = [80   100    400];



kpTorso            =   [ 30   30    30]; 
kpArms             =   [ 50   50    50   50   ];
kpLegs             =   [ 35   70    20   500   100  10]; 

kdTorso            = [ 1     1     1]; 
kdArms             = [ 1     1     1     1      ];
kdLegs             = [ 1     1     1     1     1     1]; 

kPandD_Postural    = [ kpTorso(1,:),kpArms(1,:),kpArms(1,:),kpLegs(1,:),kpLegs(1,:)
                       kdTorso(1,:),kdArms(1,:),kdArms(1,:),kdLegs(1,:),kdLegs(1,:)]*0;

                   
if (DEMO_LEFT_AND_RIGHT == 1)
    directionOfOscillation = [0;1;0];
    referenceParams        = [0.03 0.1];  %referenceParams(1) = amplitude of ascillations in meters
else                                      %referenceParams(2) = frequency of ascillations in hertz
    directionOfOscillation = [0;1;0];
    referenceParams        = [0.0  0.0];  %referenceParams(1) = amplitude of ascillations in meters
end
%% Parameters for QP
number_of_feet_on_ground = 2;
init_conditions_QP   = zeros(6*number_of_feet_on_ground,1);
lower_bound_opt_var  = -inf*ones(6*number_of_feet_on_ground,1);
upper_bound_opt_var  = inf*ones(6*number_of_feet_on_ground,1);
init_params_QP       = [init_conditions_QP;lower_bound_opt_var;upper_bound_opt_var];

%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)


% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;%1/3;  
torsionalFrictionCoefficient = 2/150;

%% The QP solver will search a solution fo that 
% satisfies the inequality Aineq_f F(fo) < bineq_f

[Aineq_fcone,bineq_fcone]= constraint_fcone_QP(forceFrictionCoefficient,numberOfPoints,number_of_feet_on_ground);


if number_of_feet_on_ground == 2
    Aineq_torsion = [ 0         , 0, -torsionalFrictionCoefficient,               0,               0, 1, zeros(1,6);
                      0         , 0, -torsionalFrictionCoefficient,               0,               0,-1, zeros(1,6); 
                      zeros(1,6), 0,              0,                -torsionalFrictionCoefficient, 0, 0,      1;
                      zeros(1,6), 0,              0,                -torsionalFrictionCoefficient, 0, 0,     -1];   
    bineq_torsion = zeros(4,1);
else
    Aineq_torsion = [ 0, 0, -torsionalFrictionCoefficient, 0, 0, 1;
                      0, 0, -torsionalFrictionCoefficient, 0, 0,-1];
    bineq_torsion = zeros(2,1); 
end    
%

% merge the constraints together
% here still in terms of f (not f0) 
% (the constraint on f0 are variable and calculated in the controller)
Aineq_F = [ Aineq_fcone;
            Aineq_torsion];
bineq_F = [ bineq_fcone;
            bineq_torsion];