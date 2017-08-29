% changeConfig
%
% A simple function for switching between different joint configurations.
% All joint configurations are stored in this function. The joint position
% is switched every time an event is triggered
%
function qj = changeConfig(t,tswitch,qj0)

% matrix of custom joint positions [deg]

                    % torso       % left arm            % right arm           % left leg                % right leg                      
customValues   = [  0   0   0     0  45   0   0   0     0 -45   0   0   0     0  45   0   0   0   0     0 -45   0   0   0   0;   % q1
                    0   0   0    30  35   0   0   0    30 -35   0   0   0    30  35   0   0  30   0    30 -35   0   0  30   0;   % q2
                    0   0   0   -30  35   0 -40   0   -30 -35   0 -40   0   -30  35   0   0 -30   0   -30 -35   0   0 -30   0;   % q3
                    0   0   0     0  45  30   0  30     0 -35 -30   0 -30     0  35  30   0   0  30     0 -35 -30   0   0 -30;   % q4
                    0   0   0     0  45 -30   0 -30     0 -35  30   0  30     0  35 -30   0   0 -30     0 -35  30   0   0  30;   % q5
                  -20   0   0     0  45 -30   0 -30     0 -35  30   0  30     0  35 -30   0   0 -30     0 -35  30   0   0  30;   % q6
                   20   0   0     0  45 -30   0 -30     0 -35  30   0  30     0  35 -30   0   0 -30     0 -35  30   0   0  30;   % q7
                   20   0 -20     0  45 -30   0 -30     0 -35  30   0  30     0  35 -30   0   0 -30     0 -35  30   0   0  30;   % q8
                   20   0  20     0  45 -30   0 -30     0 -35  30   0  30     0  35 -30   0   0 -30     0 -35  30   0   0  30;]; % q9

% add the initial joint configuration [rad]
jointPositions = [qj0 transpose(customValues)*pi/180 qj0];

% if t < tswitch(1), stay at the first configuration
qj = jointPositions(:,1);

for k = 1:length(tswitch)
    
    % if t >= tswitch(k), take the k-th configuration. Example:
    % 
    % jointPositions = [q0 q1 q2];
    % tswitch        = [10 20];
    % 
    % this means: t < 10         ---> q0 is taken
    %             10 >= t < 20   ---> q1 is taken
    %             t >= 20        ---> q2 is taken
    %
    if t >= tswitch(k)
    
        qj = jointPositions(:,k+1);
    end
    
end

end
