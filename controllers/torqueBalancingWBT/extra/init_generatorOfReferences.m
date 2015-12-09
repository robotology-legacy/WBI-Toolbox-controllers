noOscillationTime        = 10;

robotName = 'icub';
localName = 'refGen4TorqueBalancing';

outputPortCoM       = ['/', localName, '/comDes:o'];
outputPortPostural  = ['/', localName, '/qDes:o'];
balancingPort       = 'myTest';


% Controller period
Ts                = 0.01;
simulationTime    = inf;

ROBOT_DOF = 23;
directionOfOscillation  = [0;1;0];
referenceParamsCoM      = [0.06  0.2];  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz
referenceParamsPost     = [30*pi/180  2];           
mask                    = zeros(ROBOT_DOF ,1);
mask(4:end)                 = 1;                    


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
           0.2092    0.6473    0.0006   -0.1741   -0.1044    0.0700 ...
           0.3514    1.3107    1.3253   -0.0189    0.6374   -0.0614 ];
       
    q4 = [  -0.0179  0.3218    0.0076 ...
            -0.6494  1.6296    0.0013    0.8756 ...
            -0.6524  0.8722    0.0012    0.6122 ...
             0.3850  0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
             0.2091  0.2940    0.0001   -0.1738   -0.1062    0.0781 ];
   
       
  
postures = [ noOscillationTime,  q1;
             noOscillationTime+  references.smoothingTimeQDes, q2;
             noOscillationTime+4*references.smoothingTimeQDes, q3;
             noOscillationTime+5*references.smoothingTimeQDes, q4
             noOscillationTime+6*references.smoothingTimeQDes, q1;
             noOscillationTime+7*references.smoothingTimeQDes, q2;
             noOscillationTime+8*references.smoothingTimeQDes, q3;
             noOscillationTime+9*references.smoothingTimeQDes, q4
             noOscillationTime+10*references.smoothingTimeQDes, q1;
             noOscillationTime+11*references.smoothingTimeQDes, q2;
             noOscillationTime+12*references.smoothingTimeQDes, q3;
             noOscillationTime+13*references.smoothingTimeQDes, q4
             noOscillationTime+14*references.smoothingTimeQDes, q1;
             noOscillationTime+15*references.smoothingTimeQDes, q2;
             noOscillationTime+16*references.smoothingTimeQDes, q3;
             noOscillationTime+17*references.smoothingTimeQDes, q4];
         
Tt = 5;


    
