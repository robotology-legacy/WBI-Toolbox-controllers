%WBC ASW
clear all
 
setenv('YARP_ROBOT_NAME', 'iCubGenova01');
robotName = 'icub';
localName = 'balancing';
ROBOT_DOF = 23;

% Controller period
Ts             = 0.01; 
 
DEMO_MOVING_LEG_AND_ARMS = 1;
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

gainsPCOM                 = diag([ 45   45  45]);  
gainsICOM                 = diag([  0    0   0]);
gainsDCOM                 = diag([  1    1   1]);

minCoMx_y                 = [-0.1   -0.25 ];  
maxCoMx_y                 = [ 0.1    0.05 ];
satGainsPCOM              = 300;
increasingRatesGainsPCOM  = [ 0     0    ];

gainMomentum              = 1 ;

% Impadances acting in the null space of the desired contact forces 

if (DEMO_MOVING_LEG_AND_ARMS == 0)
    simulationTime = inf;
    impTorso            = [  40    40   40
                              0     0    0]; 

    impArms             = [ 15  15   15   7  
                            0    0    0   0              ];

    impLeftLeg          = [ 70  110 650     300      0   0
                             0    0   0       0      0   0]; 

    impRightLeg         = [ 40   40  20      10     10   10
                             0    0   0       0      0   0];
                         
    amplitudesOscillationsOnOneFoot   = zeros(1,ROBOT_DOF);
    frequenciesOscillationsOnOneFoot  = zeros(1,ROBOT_DOF);
                         
else
    simulationTime      = inf;
    impTorso            = [  60    60   60
                              0     0    0]; 

    impArms             = [ 15  15   15   7 
                            0    0    0   0              ];

    impLeftLeg          = [ 90  110 650     300      0   0
                             0    0   0       0      0   0]; 

    impRightLeg         = [ 70   70  20      10     10   10
                             0    0   0       0      0   0];
                         
    q1 = [-0.0790    0.2279    0.4519 ...   
          -1.1621    0.6663    0.4919    0.9947 ...  
          -1.0717    1.2904   -0.2447    1.0948 ...   
           0.2092    0.2960    0.0006   -0.1741   -0.1044    0.0700 ...
           0.3484    0.4008   -0.0004   -0.3672   -0.0530   -0.0875];
       
    q2 = [-0.0790    0.2279    0.4519 ...
          -1.1621    0.6663    0.4965    0.9947 ...
          -1.0717    1.2904   -0.2493    1.0948 ...
           0.2092    0.2960    0.0006   -0.1741   -0.1044    0.0700 ...
           0.3714    0.1906    1.3253   -0.9794    0.6374   -0.0614 ];
       
    q3 = [-0.0852   -0.4273    0.0821 ...
           0.1391    1.4585    0.2464    0.3042  ... 
          -0.4181    1.6800    0.7373    0.3031  ... 
           0.2092    0.6473    0.0006    -0.1741   -0.1044    0.0700 ...
           0.3514    1.3107    1.3253   -0.0189    0.6374   -0.0614 ];
       
%     q4 = [-0.0852    -0.4273    0.0821 ...   
%            0.1391     1.8585    0.2464    0.3042 ... 
%           -0.4181     1.2874    0.7357    0.3031 ...  
%            0.2092     0.5998    0.0006   -0.1741   -0.1167    0.0117 ...  
%            0.3499    -0.0564    1.3253   -0.9518    0.6359   -0.0614 ];
       
    postures   = [ 2, q1
                   7, q2
                  15, q3];
              
    minJerkTransitionTime = 3;
    A_f = [20 1 25];
    
end
                     
impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
impedencesSat       = [100   100    1500];



directionOfOscillation = [0;1;0];
referenceParams        = [0.0  0.0];  %referenceParams(1) = amplitude of ascillations in meters


%% Parameters for QP
number_of_feet_on_ground = 1;
init_conditions_QP   = zeros(6*number_of_feet_on_ground,1);
lower_bound_opt_var  = -inf*ones(6*number_of_feet_on_ground,1);
upper_bound_opt_var  = inf*ones(6*number_of_feet_on_ground,1);
init_params_QP       = [init_conditions_QP;lower_bound_opt_var;upper_bound_opt_var];

%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)


% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1/3;  
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

