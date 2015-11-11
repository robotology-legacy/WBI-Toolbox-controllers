ROBOT_DOF = 23;

references.directionOfOscillation  = [0;0;0];
references.amplitudeOfOscillation  = 0.0;  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz
references.frequencyOfOscillation  = 0.0;
references.noOscillationTime       = 3;    % If DEMO_LEFT_AND_RIGHT = 1, the variable noOscillationTime is the time, in seconds, 
                                            % that the robot waits before starting the left-and-righ
references.smoothingTimeQDes       = 1.0;
references.smoothingTimeComDes     = 5;

sat.torque = 24;

smoothingTimeJacobians            = 0.5;

ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);
gain.qTildeMax              = 20*pi/180;
postures = 0;  

%%
%           PARAMETERS FOR TWO FEET ONE GROUND
if (sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 2)
    gain.PCOM                 = diag([50    50  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = 2*sqrt(gain.PCOM)*0;

    gain.PAngularMomentum     = 1 ;

    % Impadances acting in the null space of the desired contact forces 

    impTorso            = [10   10   20
                            0    0    0]; 
    impArms             = [10   10    10    8   
                            0    0     0    0   ];
                        
    impLeftLeg          = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]; 

    impRightLeg         = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]; 
    
                         
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0    0  ];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0     0  0    0  0];                        
                         
                         
    if (DEMO_MOVEMENTS == 1)
        gain.PCOM                 = diag([100    50  50]);
        references.directionOfOscillation  = [0;1;0];
        references.amplitudeOfOscillation  = 0.05;
        references.frequencyOfOscillation  = 0.5;
    end 
end

% PARAMETERS FOR ONLY ONE FOOT ONE GROUND

if (sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 1)
    %%
    gain.PCOM                 = diag([50   100  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = diag([  0    0   0]);

    gain.PAngularMomentum     = 1 ;

    % Impadances acting in the null space of the desired contact forces 

    
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0    0  ];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];  
    
    scalingImp          = 1.5;
    
    impTorso            = [20   20   30
                            0    0    0]*scalingImp; 
    impArms             = [15   15    15    8   
                            0    0     0    0   ]*scalingImp;
                        
    impLeftLeg          = [ 30   30   30   120     10  10
                             0    0    0     0      0   0]*scalingImp; 

    impRightLeg         = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]*scalingImp; 
                         
    if (DEMO_MOVEMENTS == 1)
    
      q1 = [-0.0790    0.2279    0.4519 ...   
            -1.1621    0.6663    0.4919    0.9947 ...  
            -1.0717    1.2904   -0.2447    1.0948 ...   
             0.3850    0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
             0.3484    0.4008   -0.0004   -0.3672   -0.0530   -0.0875];
       
      q2 = [-0.0790    0.2279    0.4519 ...
            -1.1621    0.6663    0.4965    0.9947 ...
            -1.0717    1.2904   -0.2493    1.0948 ...
             0.3850    0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
             0.3714    0.1906    1.3253   -0.9794    0.6374   -0.0614 ];
       
      q3 = [-0.0852   -0.4273    0.0821 ...
             0.1391    1.4585    0.2464    0.3042  ... 
            -0.4181    1.6800    0.7373    0.3031  ... 
             0.3850    0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
             0.3514    1.3107    1.3253   -0.0189    0.6374   -0.0614 ];
       
      q4 = [-0.0179  0.3218    0.0076 ...
            -0.6494  1.6296    0.0013    0.8756 ...
            -0.6524  0.8722    0.0012    0.6122 ...
             0.3850  0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
             0.2091  0.2940    0.0001   -0.1738   -0.1062    0.0781 ];
   
       
  
       postures = [ references.noOscillationTime,  q1;
                    references.noOscillationTime+  references.smoothingTimeQDes, q2;
                    references.noOscillationTime+4*references.smoothingTimeQDes, q3;
                    references.noOscillationTime+5*references.smoothingTimeQDes, q4
                    references.noOscillationTime+6*references.smoothingTimeQDes, q1;
                    references.noOscillationTime+7*references.smoothingTimeQDes, q2;
                    references.noOscillationTime+8*references.smoothingTimeQDes, q3;
                    references.noOscillationTime+9*references.smoothingTimeQDes, q4
                    references.noOscillationTime+10*references.smoothingTimeQDes, q1;
                    references.noOscillationTime+11*references.smoothingTimeQDes, q2;
                    references.noOscillationTime+12*references.smoothingTimeQDes, q3;
                    references.noOscillationTime+13*references.smoothingTimeQDes, q4
                    references.noOscillationTime+14*references.smoothingTimeQDes, q1;
                    references.noOscillationTime+15*references.smoothingTimeQDes, q2;
                    references.noOscillationTime+16*references.smoothingTimeQDes, q3;
                    references.noOscillationTime+17*references.smoothingTimeQDes, q4];
    end
             
    
%%    
end

sat.integral              = 0;
gain.integral            = [intTorso,intArms,intArms,intLeftLeg,intRightLeg];
gain.impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
gain.dampings            = zeros(1,ROBOT_DOF);
gain.increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
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