%% GENERAL PARAMETERS
ROBOT_DOF                          = 23;

WBT_wbiList                        = 'ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP';

sat.torque                         = 34;
gain.qTildeMax                     = 20*pi/180; 

% Feet gains for inverse kinematics
gain.ikin.kpfeet                 = 10;
gain.ikin.kdfeet                 = 2*sqrt(gain.ikin.kpfeet);

%% PARAMETERS FOR TWO FEET ON THE GROUND
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 2)

%gain.ikin.kp               = diag([50   100  50]);
 gain.ikin.kp               = 10;
 gain.ikin.kd               = 2*sqrt(gain.ikin.kp);        
    
% Impedances acting in the null space of the desired contact forces 

 impTorso            = [ 10  5   5]; 
    
 impArms             = [ 8   8   8   12];
                        
 impLeftLeg          = [ 2   2   2    1    1.5   1]; 

 impRightLeg         = [ 2   2   2    1    1.5   1];
end

%% PARAMETERS FOR ONLY ONE FOOT ON THE GROUND
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 1)

gain.ikin.kp                 = diag([50    50  50]);
gain.ikin.kd                 = 2*sqrt(gain.ikin.kp);

% Impedances acting in the null space of the desired contact forces 
  
impTorso            = [20   20   30];

impArms             = [15   15   15   8];
                        
impLeftLeg          = [30   30   30   120  10  10];

impRightLeg         = [ 30   30   30   60  10  10];
                               
end

 gain.impedances           = [impTorso,impArms,impArms,impLeftLeg,impRightLeg];
 gain.dampings             = 0*sqrt(gain.impedances);
%gain.ikin.impedances      = gain.impedances;
 gain.ikin.impedances      = 10;
 gain.ikin.dampings        = sqrt(gain.ikin.impedances);

if (size(gain.impedances) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end

%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)
% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;
torsionalFrictionCoefficient = 2/150;

%physical size of foot
phys.footSize                = [ -0.07 0.07   ;   % xMin, xMax
                                 -0.03 0.03 ];    % yMin, yMax    
                             
gain.footSize                = [ -0.07 0.07   ;   % xMin, xMax
                                 -0.03 0.03 ];    % yMin, yMax    

fZmin                        = 10;

%% Regulation terms 
reg.pinvTol     = 1e-8;
reg.pinvDamp    = 1e-6;
reg.pinvDampVb  = 0.001;
