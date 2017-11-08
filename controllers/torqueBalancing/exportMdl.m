%% EXPORTMDL 
%  automatic exports .mdl files in previous Simulink versions for keeping 
%  the back-compatibility of the controllers. 
% 
%  - supported Matlab versions: 2016b, 2015b, 2015a, 2014a
%
%  - this script should be launched at least from Matlab version 2014a
%
%  - adapted from https://github.com/robotology/WB-Toolbox/blob/master/toolbox/export_library.m
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, April 2017
%

%% Initialization
clc
clear variables 
close all

fprintf('\n### Exporting Simulink model to previous versions');
fprintf('\n### Matlab version must be at least 2014a');
fprintf('\n### Supported versions: 2016b, 2015b, 2015a, 2014a');
fprintf('\n### If you need to export from other versions, just modify this script accordingly.\n');

% at the moment, this script can export models only from a Matlab version 
% greater than 2014a
if (verLessThan('matlab', '8.3'))
    error('This script should be launched with a MATLAB version >= than 2014a');
    quit; %#ok<UNRCH>
end

%% Find the Matlab version in use
matlabVersion     = ver('matlab');
matlabVersion     = matlabVersion.Version;

% prefix associated to the model to export
if str2double(matlabVersion) == 8.3
    versionName   = 'R2014a';
elseif str2double(matlabVersion) == 8.5
    versionName   = 'R2015a';    
elseif str2double(matlabVersion) == 8.6   
    versionName   = 'R2015b';  
elseif str2double(matlabVersion) == 9.1   
    versionName   = 'R2016b';  
end

% name of the model to be loaded and exported
modelName         = ['torqueBalancing' versionName];

%% Load the Simulink model and export to previous versions

% if Matlab version is 2014a, there's nothing to export
if str2double(matlabVersion) == 8.3
    
    fprintf('\nMatlab version is 2104a.\n');
    fprintf('\nNothing to export.\n');
else
    try  
        open_system(modelName,'loadonly');
        fprintf(['\nLOADED MODEL: ' modelName '\n']);

        fprintf('\nExporting to version 2014a\n');
        save_system(modelName, 'torqueBalancingR2014a', 'ExportToVersion', 'R2014A_MDl');

        if str2double(matlabVersion) >= 8.6  
            % export to 2015a
            fprintf('\nExporting to version 2015a\n');
            save_system(modelName, 'torqueBalancingR2015a', 'ExportToVersion', 'R2015A_MDl');
        end 
        if str2double(matlabVersion) >= 9.1    
            % export to 2015b
            fprintf('\nExporting to version 2015b\n'); 
            save_system(modelName, 'torqueBalancingR2015b', 'ExportToVersion', 'R2015B_MDl');
        end
        % closing the model
        close_system(modelName);
        catch ex;
    end
end
    
fprintf('\nDone\n');
