noOscillationTime        = 5;

robotName = 'icub';
localName = 'refGen4TorqueBalancing';

outputPortCoM       = ['/', localName, '/comDes:o'];
outputPortPostural  = ['/', localName, '/qDes:o'];
balancingPort       = 'myTest';


% Controller period
Ts                = 0.01;
simulationTime    = inf;

ROBOT_DOF = 15;
directionOfOscillation  = [0;1;0];
referenceParamsCoM      = [0.03  0.05];  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz
referenceParamsPost     = [30*pi/180  2];           
mask                    = zeros(ROBOT_DOF ,1);
mask(4:end)                 = 1;                    



       
q1 = [-0.0790    0.2279    0.4519 ...
       0.2092    0.2960    0.0006   -0.1741   -0.1044    0.0700 ...
       0.3714    0.1906    1.1253   -0.9794    0.6374   -0.0614 ];
       
q2 = [-0.0852   -0.4273    0.0821 ...
       0.2092    0.6473    0.0006    -0.1741   -0.1044    0.0700 ...
       0.3514    1.3107    1.1253   -0.0189    0.6374   -0.0614 ];
   
postures = [ noOscillationTime,  q1;
             10+noOscillationTime, q2];
         
Tt = 2.5;