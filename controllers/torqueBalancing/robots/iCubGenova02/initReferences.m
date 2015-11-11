references.directionOfOscillation  = [0;0;0];
references.amplitudeOfOscillation  = 0.0;  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz
references.frequencyOfOscillation  = 0.0;
references.noOscillationTime       = 3;    % If DEMO_LEFT_AND_RIGHT = 1, the variable noOscillationTime is the time, in seconds, 
                                            % that the robot waits before starting the left-and-righ
references.smoothingTimeQDes       = 1.0;
references.smoothingTimeComDes     = 5;


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
        
clear q1 q2 q3 q4;
    