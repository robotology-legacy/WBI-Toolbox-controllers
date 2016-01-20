ROBOT_DOF = 15;


references.directionOfOscillation  = [0;0;0];
references.amplitudeOfOscillation  = 0.0;  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz
references.frequencyOfOscillation  = 0.0;
references.noOscillationTime       = 0; % If DEMO_LEFT_AND_RIGHT = 1, the variable noOscillationTime is the time, in seconds, 
                                       % that the robot waits before starting the left-and-righ

 
sat.torque = 24;

smoothingTimeJacobians            = 0.5;

ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);
gain.qTildeMax              = 20*pi/180;

%%
%           PARAMETERS FOR TWO FEET ONE GROUND
if (sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 2)
    gain.PCOM                 = diag([ 80   80   80]);
    gain.ICOM                 = diag([  0    0    0]);
    gain.DCOM                 = 0*sqrt(gain.PCOM);

    gain.PAngularMomentum     = 1;

    % Impadances acting in the null space of the desired contact forces 


    % 
    impTorso            = [   60    60   10
                               0     0    0]*5; 
                        
    impLeftLeg          = [ 35   20    30     150     50   0
                             0    0     0       0      0   0]; 

    impRightLeg         = [35   20    30      150     50   0
                            0    0     0        0      0   0]; 
    
                         
    intTorso            = [0   0    0]; 
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];                        
                         
                         
    if (DEMO_MOVEMENTS == 1)
        references.directionOfOscillation  = [0;1;0];
        references.amplitudeOfOscillation  = 0.03;
        references.frequencyOfOscillation  = 0.75;
    end
    
end

%%
%           PARAMETERS FOR ONLY ONE FOOT ONE GROUND


if (sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 1)
    %%
    gain.PCOM                 = diag([120  140 120])/3;
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = diag([  1    1   1]);

    gain.PAngularMomentum     = 0.5 ;

    % Impadances acting in the null space of the desired contact forces 

    
    intTorso            = [0   0    0]; 
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];  
    
    impTorso            = [  20    30   20
                              0     0    0]*5; 

    impLeftLeg          = [ 100   70  65      30     20  20
                             0    0   0       0      0   0]; 

    impRightLeg         = [ 30   30  30      30     40   40
                             0    0   0       0      0   0];

   
end

sat.integral              = 0;
gain.integral             = [intTorso,intLeftLeg,intRightLeg];
gain.impedances           = [impTorso(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
gain.dampings             = zeros(1,ROBOT_DOF);
gain.increasingRatesImp   = [impTorso(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
sat.impedences            = [80   25    1400];

if (size(gain.impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end


%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)


% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;%1/3;  
torsionalFrictionCoefficient = 2/150;

gain.footSize                = [ -0.1 0.1   ;    % xMin, xMax
                                 -0.1 0.1  ];   % yMin, yMax    

fZmin                        = 10;

%% The QP solver will search a solution fo that 
% satisfies the inequality Aineq_f F(fo) < bineq_f
reg.pinvTol     = 1e-5;
reg.pinvDamp    = 0.01;
reg.HessianQP   = 1e-7;