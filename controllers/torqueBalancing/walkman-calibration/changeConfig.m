% changeConfig
%
% A simple function for switching between different joint configurations.
% All joint configurations are stored in this function. The joint position
% is switched every time a time event is triggered
%
function qj = changeConfig(t,tswitch,qj0)

jointPositions = qj0;
jointPositions = [jointPositions transpose([0 0 0   0 30 0 0 0   0 -30 0 0 0   0 30 0 0 0 0   0 -30 0 0 0 0])];
jointPositions = jointPositions*pi/180;

qj = jointPositions(:,1);

for k = 1:length(tswitch)
    
    if t >= tswitch(k)
    
        qj = jointPositions(:,k+1);
    end
    
end

end
