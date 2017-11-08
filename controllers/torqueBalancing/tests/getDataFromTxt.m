%% Convert .txt data into a matlab format

% open the file
fid = fopen('test.txt');

% parse the lines in the .txt
res  = {};

while ~feof(fid)

   res{end+1} = fgetl(fid); %#ok<SAGROW>
end

% from the second line, convert lines from char to double
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

%% Plot check
% figure(1)
% 
% hold on
% grid on
% 
% plot(qjDes)

%% Close file
fclose(fid);

% joint sequence:
%
% torso_pitch torso_roll torso_yaw 
% l_hip_pitch l_hip_roll l_hip_yaw l_knee l_ankle_pitch l_ankle_roll 
% r_hip_pitch r_hip_roll r_hip_yaw r_knee r_ankle_pitch r_ankle_roll 

% data for Simulink
simulinkData = [time qjDes];

 