ROBOT_DOF = 23;

references.directionOfOscillation  = [0;0;0];
references.amplitudeOfOscillation  =  0.0;          %referenceParams(1) = amplitude of ascillations in meters 
references.frequencyOfOscillation  =  0.0;          %referenceParams(2) = frequency of ascillations in hertz

references.noOscillationTime       = 0;    % If DEMO_MOVEMENTS = 1, the variable noOscillationTime is the time, in seconds, 
                                           % that the robot waits before starting the left-and-righ
                                       
smoothingTimePostures = 3;

sat.torque = 24;

smoothingTimeJacobians            = 0.5;

ROBOT_DOF_FOR_SIMULINK            = eye(ROBOT_DOF);
gain.qTildeMax                    = 20*pi/180;

gain.ikin.kp  = 5;
gain.ikin.kd  = 2*sqrt(gain.ikin.kp);

%%
% PARAMETERS FOR TWO FEET ONE GROUND
if (sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 2)

% Impedances acting in the null space of the desired contact forces 

 impTorso            = [  15   5  10
                          0    0   0]; 
    
 impArms             = [ 8   8    8   12    
                         0   0    0    0 ];
                        
 impLeftLeg          = [ 2   2    2     2      2   1
                         0   0    0     0      0   0]*2; 

 impRightLeg         = [2    2     2    2      2   1
                        0    0     0    0      0   0]*2;
    
    
dampTorso            = [  0    0    0];

dampArms             = [ 0   0    0    0 ];
                        
dampLeftLeg          = [ 0    0     0       0      2*sqrt(impLeftLeg(1,5))   0]; 

dampRightLeg         = [ 0    0     0       0      2*sqrt(impLeftLeg(1,5))   0]; 
    
                         
intTorso             = [0   0    0];

intArms              = [0   0    0    0  ];
                        
intLeftLeg           = [0   0    0    0    0  0]; 

intRightLeg          = [0   0    0    0    0  0];                        
                         
                         
    if (DEMO_MOVEMENTS == 1)
        references.directionOfOscillation  = [0;1;0];
        references.amplitudeOfOscillation  = 0.05;  
        references.frequencyOfOscillation  = 0.1;
    end
end

%%
%PARAMETERS FOR ONLY ONE FOOT ONE GROUND


if (sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 1)

    % Impedances acting in the null space of the desired contact forces 

    intTorso            = [0   0    0];
    
    intArms             = [0   0    0    0  ];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];  
    

    impTorso            = [  20    20   20
                              0     0    0]; 

    impArms             = [ 13  13   13   5  
                            0    0    0   0  ];

    impLeftLeg          = [ 70   70  65    30      0   0
                             0    0   0     0      0   0]; 

    impRightLeg         = [ 20   20  20    10      0    0
                             0    0   0     0      0   0];
                         
                         
    dampTorso            = [  0    0    0
                              0    0    0];
                          
    dampArms             = [ 0   0    0    0   
                             0   0    0    0];
                        
    dampLeftLeg          = [ 0    0     0       0      2*sqrt(impLeftLeg(1,5))   0
                             0    0     0       0      0   0]; 

    dampRightLeg         = [ 0    0     0       0      2*sqrt(impLeftLeg(1,5))   0
                             0    0     0       0      0   0];    
end

%%
sat.integral              = 0;
gain.integral             = [intTorso,intArms,intArms,intLeftLeg,intRightLeg];
gain.impedances           = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
gain.dampings             = 0*[dampTorso,dampArms,dampArms,dampLeftLeg,dampRightLeg];
gain.increasingRatesImp   = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
sat.impedences            = [80   25    1400];

if (size(gain.impedances,2) ~= ROBOT_DOF || size(gain.dampings,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end


%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)


% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;%1/3;  
torsionalFrictionCoefficient = 2/150;

gain.footSize               = [ -0.1 0.1   ;   % xMin, xMax
                                 -0.1 0.1  ];   % yMin, yMax    

fZmin                        = 10;

%% The QP solver will search a solution fo that 
% satisfies the inequality Aineq_f F(fo) < bineq_f
reg.pinvTol     = 5e-6;
reg.pinvDamp    = 5e-6;
reg.HessianQP   = 0;
reg.pinvDampVb  = 0.001;



