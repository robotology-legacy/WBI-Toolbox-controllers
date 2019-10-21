% loadJointConfiguration
%
% this function acts as a storage. It constains the joint positions for 
% different calibration movesets on robot Walkman.
%
function jointPos = loadJointConfiguration(qj0,CONFIG)

jointPos = qj0;

% select a moveset
if CONFIG.moveset == 1
    
                        % torso       % left arm            % right arm           % left leg                % right leg        
    customValues   = [ 0  0   0      0  30  0  0  0      0  -30   0   0  0     0  30   0 0   0    0       0  -30   0  0   0   0;   % q1
                       0  0   0     20  30  0  0  0     20  -30   0   0  0     20 30   0 0   20   0      20  -30   0  0   20  0;   % q2
                       0  0   0    -20  30  0  0  0    -20  -30   0   0  0    -20 30   0 0  -20   0      20  -30   0  0  -20  0;   % q3
                      10  0   0    -20  30  10 0  0    -20  -30  10   0  0    -20 30  10 0  -20  20     -20  -30  10  0  -20  20;  % q4
                     -10  0   0    -20  30 -10 0  0    -20  -30 -10   0  0    -20 30 -10 0  -20 -20     -20  -30 -10  0  -20 -20;  % q5
                       0  0   0      0  40  0  20 0      0  -40   0  20  0     0  40  0 -20  0    0       0  -40  0  -20  0   0;   % q6
                       0  0   0      0  40  0 -20 0      0  -40   0 -20  0     0  40  0 -20  0    0       0  -40  0  -20  0   0;   % q7
                       0  0   15     0  30  0  0  10     0  -30   0   0  10    0  30  0  0   0    0       0  -30  0   0   0   0;   % q8
                       0  0  -15     0  30  0  0 -10     0  -30   0   0 -10    0  30  0  0   0    0       0  -30  0   0   0   0;]; % q9
     
    % add the initial joint configuration and convert to radians
    jointPos = [qj0 transpose(customValues)*pi/180 qj0];
    
elseif CONFIG.moveset == 2
    
                        % torso       % left arm            % right arm           % left leg                % right leg                      
    customValues   = [ 0  0   0     0  45   0   0   0     0 -45   0   0   0     0  45   0   0   0   0     0 -45   0   0   0   0;   % q1
                       0  0   0    30  35   0   0   0    30 -35   0   0   0    30  35   0   0  30   0    30 -35   0   0  30   0;   % q2
                       0  0   0   -30  35   0 -40   0   -30 -35   0 -40   0   -30  35   0   0 -30   0   -30 -35   0   0 -30   0;   % q3
                       0  0   0     0  45  30   0  30     0 -35 -30   0 -30     0  35  30  30   0  30     0 -35 -30  30   0 -30;   % q4
                       0  0   0     0  45 -30   0 -30     0 -35  30   0  30     0  35 -30  30   0 -30     0 -35  30  30   0  30;   % q5
                     -10  0   0     0  45 -30   0 -30     0 -35  30   0  30     0  35 -30   0   0 -30     0 -35  30   0   0  30;   % q6
                      10  0   0     0  45 -30   0 -30     0 -35  30   0  30     0  35 -30   0   0 -30     0 -35  30   0   0  30;   % q7
                      10  0 -10     0  45 -30   0 -30     0 -35  30   0  30     0  35 -30   0   0 -30     0 -35  30   0   0  30;   % q8
                      10  0  10     0  45 -30   0 -30     0 -35  30   0  30     0  35 -30   0   0 -30     0 -35  30   0   0  30;]; % q9
     
     % add the initial joint configuration and convert to radians             
     jointPos = [qj0 transpose(customValues)*pi/180 qj0];                 
    
elseif CONFIG.moveset == 3
    
                     % torso      % left arm         % right arm          % left leg              % right leg      
    customValues = [-10, 0,  0,   0, 45, 0, 0, 0,    0,   0, 0, 0, 0,      0, -10, 0, 0, 0, 10,     0,  -5, 0, 0, 0,-15;
                    -10, 0,  0,   0,  0, 0, 0, 0,    0, -45, 0, 0, 0,      0,   5, 0, 0, 0, 15,     0,  10, 0, 0, 0, 10];  %% State 2 CoM transition to left
                    
       
      % add the initial joint configuration and convert to radians             
     jointPos = [qj0 transpose(customValues(1,:))*pi/180 qj0 transpose(customValues(2,:))*pi/180 qj0];  
        
end
end

