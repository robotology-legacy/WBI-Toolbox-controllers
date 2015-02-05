function [trajectories, splittedParts] = TrajectoryLoader(filename, splitParts, newSampleTime)

shouldSplitIntoParts = 0;
shouldResample = 0;
if nargin > 1
    shouldSplitIntoParts = splitParts;
end

if nargin > 2
    shouldResample = 1;
end

splittedParts = containers.Map();

%% Import data from text file.
delimiter = ',';
startRow = 2;

formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.


if shouldResample == 1;
    %%Resample trajectories
    %%TODO
end

%% Allocate imported array to column variable names
rootiCubt1 = dataArray{:, 1};
rootiCubtau_l_hip_pitch1 = dataArray{:, 2};
rootiCubtau_l_hip_roll1 = dataArray{:, 3};
rootiCubtau_l_hip_yaw1 = dataArray{:, 4};
rootiCubtau_l_knee1 = dataArray{:, 5};
rootiCubtau_l_ankle_pitch1 = dataArray{:, 6};
rootiCubtau_l_ankle_roll1 = dataArray{:, 7};
rootiCubtau_r_hip_pitch1 = dataArray{:, 8};
rootiCubtau_r_hip_roll1 = dataArray{:, 9};
rootiCubtau_r_hip_yaw1 = dataArray{:, 10};
rootiCubtau_r_knee1 = dataArray{:, 11};
rootiCubtau_r_ankle_pitch1 = dataArray{:, 12};
rootiCubtau_r_ankle_roll1 = dataArray{:, 13};
rootiCubtau_torso_pitch1 = dataArray{:, 14};
rootiCubtau_torso_roll1 = dataArray{:, 15};
rootiCubtau_torso_yaw1 = dataArray{:, 16};
rootiCubtau_l_shoulder_pitch1 = dataArray{:, 17};
rootiCubtau_l_shoulder_roll1 = dataArray{:, 18};
rootiCubtau_l_shoulder_yaw1 = dataArray{:, 19};
rootiCubtau_l_elbow1 = dataArray{:, 20};
rootiCubtau_l_wrist_prosup1 = dataArray{:, 21};
rootiCubtau_l_wrist_pitch1 = dataArray{:, 22};
rootiCubtau_l_wrist_yaw1 = dataArray{:, 23};
rootiCubtau_neck_pitch1 = dataArray{:, 24};
rootiCubtau_neck_roll1 = dataArray{:, 25};
rootiCubtau_neck_yaw1 = dataArray{:, 26};
rootiCubtau_r_shoulder_pitch1 = dataArray{:, 27};
rootiCubtau_r_shoulder_roll1 = dataArray{:, 28};
rootiCubtau_r_shoulder_yaw1 = dataArray{:, 29};
rootiCubtau_r_elbow1 = dataArray{:, 30};
rootiCubtau_r_wrist_prosup1 = dataArray{:, 31};
rootiCubtau_r_wrist_pitch1 = dataArray{:, 32};
rootiCubtau_r_wrist_yaw1 = dataArray{:, 33};
rootiCubq_l_hip_pitch1 = dataArray{:, 34};
rootiCubq_l_hip_roll1 = dataArray{:, 35};
rootiCubq_l_hip_yaw1 = dataArray{:, 36};
rootiCubq_l_knee1 = dataArray{:, 37};
rootiCubq_l_ankle_pitch1 = dataArray{:, 38};
rootiCubq_l_ankle_roll1 = dataArray{:, 39};
rootiCubq_r_hip_pitch1 = dataArray{:, 40};
rootiCubq_r_hip_roll1 = dataArray{:, 41};
rootiCubq_r_hip_yaw1 = dataArray{:, 42};
rootiCubq_r_knee1 = dataArray{:, 43};
rootiCubq_r_ankle_pitch1 = dataArray{:, 44};
rootiCubq_r_ankle_roll1 = dataArray{:, 45};
rootiCubq_torso_pitch1 = dataArray{:, 46};
rootiCubq_torso_roll1 = dataArray{:, 47};
rootiCubq_torso_yaw1 = dataArray{:, 48};
rootiCubq_l_shoulder_pitch1 = dataArray{:, 49};
rootiCubq_l_shoulder_roll1 = dataArray{:, 50};
rootiCubq_l_shoulder_yaw1 = dataArray{:, 51};
rootiCubq_l_elbow1 = dataArray{:, 52};
rootiCubq_l_wrist_prosup1 = dataArray{:, 53};
rootiCubq_l_wrist_pitch1 = dataArray{:, 54};
rootiCubq_l_wrist_yaw1 = dataArray{:, 55};
rootiCubq_neck_pitch1 = dataArray{:, 56};
rootiCubq_neck_roll1 = dataArray{:, 57};
rootiCubq_neck_yaw1 = dataArray{:, 58};
rootiCubq_r_shoulder_pitch1 = dataArray{:, 59};
rootiCubq_r_shoulder_roll1 = dataArray{:, 60};
rootiCubq_r_shoulder_yaw1 = dataArray{:, 61};
rootiCubq_r_elbow1 = dataArray{:, 62};
rootiCubq_r_wrist_prosup1 = dataArray{:, 63};
rootiCubq_r_wrist_pitch1 = dataArray{:, 64};
rootiCubq_r_wrist_yaw1 = dataArray{:, 65};

trajectories = [
    rootiCubt1, ...
    rootiCubq_torso_pitch1, ...
    rootiCubq_torso_roll1, ...
    rootiCubq_torso_yaw1, ...
    rootiCubq_l_shoulder_pitch1, ...
    rootiCubq_l_shoulder_roll1, ...
    rootiCubq_l_shoulder_yaw1, ...
    rootiCubq_l_elbow1, ...
    rootiCubq_l_wrist_prosup1, ...
    rootiCubq_r_shoulder_pitch1, ...
    rootiCubq_r_shoulder_roll1, ...
    rootiCubq_r_shoulder_yaw1, ...
    rootiCubq_r_elbow1, ...
    rootiCubq_r_wrist_prosup1 ...
    rootiCubq_l_hip_pitch1, ...
    rootiCubq_l_hip_roll1, ...
    rootiCubq_l_hip_yaw1, ...
    rootiCubq_l_knee1, ...
    rootiCubq_l_ankle_pitch1, ...
    rootiCubq_l_ankle_roll1, ...
    rootiCubq_r_hip_pitch1, ...
    rootiCubq_r_hip_roll1, ...
    rootiCubq_r_hip_yaw1, ...
    rootiCubq_r_knee1, ...
    rootiCubq_r_ankle_pitch1, ...
    rootiCubq_r_ankle_roll1   
];

if shouldSplitIntoParts == 1;
    %%Split into one var for each part
    auxiliary = [ ...
        (0:length(rootiCubt1)-1)', ...
        rootiCubt1 ...
        ];
    
    torso = [ ...
        rootiCubq_torso_pitch1, ...
        rootiCubq_torso_roll1, ...
        rootiCubq_torso_yaw1 ...
        ];
    l_arm = [ ...
        rootiCubq_l_shoulder_pitch1, ...
        rootiCubq_l_shoulder_roll1, ...
        rootiCubq_l_shoulder_yaw1, ...
        rootiCubq_l_elbow1, ...
        rootiCubq_l_wrist_prosup1 ...
        ];
    
    r_arm =[ ...
        rootiCubq_r_shoulder_pitch1, ...
        rootiCubq_r_shoulder_roll1, ...
        rootiCubq_r_shoulder_yaw1, ...
        rootiCubq_r_elbow1, ...
        rootiCubq_r_wrist_prosup1 ...
        ];
    l_leg = [ ...
        rootiCubq_l_hip_pitch1, ...
        rootiCubq_l_hip_roll1, ...
        rootiCubq_l_hip_yaw1, ...
        rootiCubq_l_knee1, ...
        rootiCubq_l_ankle_pitch1, ...
        rootiCubq_l_ankle_roll1 ...
        ];
    r_leg = [ ...
        rootiCubq_r_hip_pitch1, ...
        rootiCubq_r_hip_roll1, ...
        rootiCubq_r_hip_yaw1, ...
        rootiCubq_r_knee1, ...
        rootiCubq_r_ankle_pitch1, ...
        rootiCubq_r_ankle_roll1 ...
        ];
    
    varname=@(x) inputname(1);
    splittedParts(varname(auxiliary)) = auxiliary;
    splittedParts(varname(torso)) = torso;
    splittedParts(varname(l_arm)) = l_arm;
    splittedParts(varname(r_arm)) = r_arm;
    splittedParts(varname(l_leg)) = l_leg;
    splittedParts(varname(r_leg)) = r_leg;
end

%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;