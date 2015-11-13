references.joints.smoothingTime    = 3.0;
references.com.smoothingTime       = 3;
sm.com.threshold   = 0.01;
sm.wrench.threshold = 50;

gain.PCOM              = diag([50    70  50]);
gain.ICOM              = diag([  0    0   0]);
gain.DCOM              = 2*sqrt(gain.PCOM)*0;

gain.PAngularMomentum  = 1 ;

% Impadances acting in the null space of the desired contact forces 



gain.impedances        = [10   10   20, 10   10    10    8, 10   10    10    8, 30   30   20   200     10  10, 30   50   30    60     35  50
                          10   10   20, 10   10    10    8, 10   10    10    8, 30   30   20   200     10  10, 30   50   30    60     35  50
                          10   10   20, 10   10    10    8, 10   10    10    8, 30   30   20   200     10  10, 30   50   30    60     35  50
                          10   10   20, 10   10    10    8, 10   10    10    8, 30   30   20   200     10  10, 30   50   30    60     35  50
                          10   10   20, 10   10    10    8, 10   10    10    8, 30   30   20   200     10  10, 30   50   30    60     35  50];



state = 0;




sm.com.states      = [0,0.0,0.511;
                      0,0.0,0.511;
                      0,0.0,0.511
                      0,-0.09,0.511
                      0,-0.09,0.511];

sm.tBalancing      = 1;

sm.DT              = 0;

sm.joints.states = [[
   -0.0870
    0.1362
    0.0720
   -0.6248
    0.5205
    0.0044
    0.8756
   -0.6233
    0.5240
    0.0027
    0.8699
    0.2761
   -0.1892
   -0.0078
   -0.2313
   -0.0377
    0.1875
    0.1370
    0.3815
    0.0001
   -0.1815
   -0.1016
   -0.2946]';
   [
   -0.0870
    0.1362
    0.0720
   -0.6248
    0.5205
    0.0044
    0.8756
   -0.6233
    0.5240
    0.0027
    0.8699
    0.2761
   -0.1892
   -0.0078
   -0.2313
   -0.0377
    0.1875
    0.1370
    0.3815
    0.0001
   -0.1815
   -0.1016
   -0.2946]';
   [
   -0.0870
    0.1362
    0.0720
   -0.6248
    0.5205
    0.0044
    0.8756
   -0.6233
    0.5240
    0.0027
    0.8699
    0.2761
   -0.1892
   -0.0078
   -0.2313
   -0.0377
    0.1875
    0.1370
    0.3815
    0.0001
   -0.1815
   -0.1016
   -0.2946]'];

q1 = [-0.0790    0.2279    0.4519 ...   
    -1.1621    0.6663    0.4919    0.9947 ...  
    -1.0717    1.2904   -0.2447    1.0948 ...   
     0.3850    0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
     0.3484    0.4008   -0.0004   -0.3672   -0.0530   -0.0875];

q2 = [-0.0790    0.2279    0.4519 ...
    -1.1621    0.6663    0.4965    0.9947 ...
    -1.0717    1.2904   -0.2493    1.0948 ...
     0.3850    0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
     0.2858    1.2190    1.2227   -1.7446    0.1469   -0.3468 ];

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



sm.joints.points = [ 0,q1;
                     references.joints.smoothingTime,q2;
                     2*references.joints.smoothingTime,q3;
                     6*references.joints.smoothingTime,q4];

clear q1 q2 q3 q4;
