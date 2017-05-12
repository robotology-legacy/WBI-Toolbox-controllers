%% Data from Walkman on the pole: joint positions, motor positions
% joints positions [rad]
   jointPos = [4.7010e-03
              -4.2510e-02
              -7.1585e-01
               1.1717e+00
              -4.2782e-01
              -7.9430e-03
              -1.3173e-02
               3.5866e-03
              -6.9060e-01
               1.0796e+00
              -4.0230e-01
               1.2873e-02
              -2.6389e-02
              -2.3688e-01
               1.9953e-02
               2.6635e-01
               1.4610e+00
               5.9412e-02
              -4.2040e-01
              -3.0032e-02
               4.5498e-01
               1.4115e-01
               3.5222e-02
               1.2132e-01
               2.4130e-01
              -1.4437e+00
               4.9266e-04
              -4.2004e-01
              -4.6487e-02
               3.8245e-01
              -1.5146e-01];
  
% motor positions [rad]
 motorPos = [ 0.0088178
             -0.0431275
             -0.7163785
              1.1687161
             -0.4282428
             -0.0076555
             -0.0132033
              0.0028137
             -0.6908523
              1.0755738
             -0.4021622
              0.0117828
             -0.0264488
             -0.2356984
              0.0207023
              0.2682971
              1.4642942           
              0.0615447
             -0.4247006
             -0.0331364
              0.4586962
              0.1466749
              0.0352216
              0.1213163
              0.2406376
             -1.4424423
              0.0020979
             -0.4243385
             -0.0464868
              0.3866231
             -0.1527749 ];
         
% joint list map
 jointList = {
 'LHipLat  ID: 0'
 'LHipYaw  ID: 1'
 'LHipSag  ID: 2'
 'LKneeSag  ID: 3'
 'LAnkSag  ID: 4'
 'LAnkLat  ID: 5'
 'RHipLat  ID: 6'
 'RHipYaw  ID: 7'
 'RHipSag  ID: 8'
 'RKneeSag  ID: 9'
 'RAnkSag  ID: 10'
 'RAnkLat  ID: 11'
 'WaistLat  ID: 12'
 'WaistSag  ID: 13'
 'WaistYaw  ID: 14'
 'LShSag  ID: 15'
 'LShLat  ID: 16'
 'LShYaw  ID: 17'
 'LElbj  ID: 18'
 'LForearmPlate  ID: 19'
 'LWrj1  ID: 20'
 'LWrj2  ID: 21'
 'NeckYawj  ID: 22'
 'NeckPitchj  ID: 23'
 'RShSag  ID: 24'
 'RShLat  ID: 25'
 'RShYaw  ID: 26'
 'RElbj  ID: 27'
 'RForearmPlate  ID: 28'
 'RWrj1  ID: 29'
 'RWrj2  ID: 30'};

%% Desired joint position for the upper body while the robot is on the pole 
% the references are the positions and orientations of the two feet (end
% effectors) w.r.t. the robot pelvis 
% 
desiredLegsPose = [0.00416779;
                    0.0;
                   -0.6896;
                    1.09838;
                   -0.408784;
                   -0.00461779;
                   -0.00416779;
                    0.0;
                   -0.6896;
                    1.09838;
                   -0.408784;
                    0.00461779];

% we are going to compare this position with the one of the legs motor pose        
fprintf('           COMPARISON [deg]:\n\n')
disp('   Desired    Real      Error')
disp([desiredLegsPose motorPos(1:12) (desiredLegsPose-motorPos(1:12))]*180/pi)                
          
                   
