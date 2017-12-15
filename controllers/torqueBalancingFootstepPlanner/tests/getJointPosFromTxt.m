%% Extracts from .txt file the joint positions for walking (a specific format 
%% for data is required).

% open the .txt file
fid = fopen('jointPosForWalking.txt');

% parse all the lines in the .txt file
res  = {};

while ~feof(fid)

   res{end+1} = fgetl(fid); %#ok<SAGROW>
end

% starting from the second line, convert lines from char to double
cuttingStep = 52;

step  = zeros(length(res)-1-cuttingStep,1);
time  = zeros(length(res)-1-cuttingStep,1);
qjDes = zeros(length(res)-1-cuttingStep,15);

for k=2:length(res)-cuttingStep

    splittedString = strsplit(res{1,k});
    
    % expected length: 17. If longer, cut it
    if length(splittedString) > 17
        
        % warning('Row length mismatch. Fixing it...');
        splittedString = splittedString(1:17);
    end
    
    step(k-1,:)  = str2double(splittedString(1));
    time(k-1,:)  = str2double(splittedString(2));
    qjDes(k-1,:) = str2double(splittedString(3:end));
end

% close file
fclose(fid);

%% Plot check
% figure(1)
% 
% hold all
% grid on
% plot(time,qjDes(:,end-2:end))

%% Save data into a matlab variable
% joint sequence:
%
% torso_pitch torso_roll torso_yaw 
% l_hip_pitch l_hip_roll l_hip_yaw l_knee l_ankle_pitch l_ankle_roll 
% r_hip_pitch r_hip_roll r_hip_yaw r_knee r_ankle_pitch r_ankle_roll 

% data for Simulink
jointDesiredForSimulink = [time qjDes];

 