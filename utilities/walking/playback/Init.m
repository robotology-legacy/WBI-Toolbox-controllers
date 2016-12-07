%% load trajectories
trajectories = TrajectoryLoader([pwd,filesep,'softTouchDown.csv']);

%% parameters
Ts = 0.006;
endTime = trajectories(end,1);
robotName  = 'icubSim';
localName = 'simulink'; 
ROBOT_DOF = 25;
blendTime = 2;

%% add some row with constant initial positions
holdPositionTime = 3;
sampleTime = trajectories(3,1)-trajectories(2,1);
numberOfRow = holdPositionTime/sampleTime;

for i=1:numberOfRow
    trajectories = [trajectories(1,:); trajectories];
end

numberOfSamples = length(trajectories(:,1));

for i = 1 : numberOfSamples
    trajectories(i,1) = (i-1)*sampleTime;
end

