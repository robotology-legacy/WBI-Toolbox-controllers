% changeConfig
%
% A simple function for switching between different joint configurations.
% The joint position is switched every time an event (tswitch) is triggered.
%
function qj = changeConfig(t,tswitch,qj0,CONFIG)

% call the function which stores the joint positions
jointPositions = loadJointConfiguration(qj0,CONFIG);

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
