%% EXPORTMDL 
%  automatic exports .mdl files in previous Simulink versions for keeping 
%  the back-compatibility of the controllers. 
% 
%  - available Matlab versions: 2016b, 2015b, 2015a, 2012b
%
%  - this script should be launched at least from Matlab version 2015a
%
%  - adapted from https://github.com/robotology/WB-Toolbox/blob/master/toolbox/export_library.m
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, April 2017
%

%% Initialization
clc
clear 
close

fprintf('\n### Exporting Simulink model to previous versions');
fprintf('\n### Matlab version must be at least 2015a');
fprintf('\n### Exported versions: 2015b, 2015a, 2012b');
fprintf('\n### If you need to export other versions, just modify this script accordingly.\n');

% at the moment, this script can export models only from a Matlab version 
% greater than 2015a
if (verLessThan('matlab', '8.5'))
    error('This script should be launched with a MATLAB version >= than 2015a');
    quit; %#ok<UNRCH>
end

%% find the Matlab version
matlabVersion     = ver('matlab');
matlabVersion     = matlabVersion.Version;
% generate the prefix for loading the model
if str2double(matlabVersion) == 8.5
    versionName   = 'R2015aS85';
elseif str2double(matlabVersion) < 9.1 && str2double(matlabVersion) >= 8.6
    versionName   = 'R2015bS86';    
elseif str2double(matlabVersion) >= 9.1   
    versionName   = 'R2016bS88';  
end
modelName         = ['torqueBalancing' versionName];

%% Load the Simulink model and export to previous versions
try
  open_system(modelName,'loadonly');
  fprintf(['\nLOADED MODEL: ' modelName '\n']);

  % export to 2012b
  fprintf('\nExporting to version 2012b\n');
  save_system(modelName, 'torqueBalancingR2012bS80', 'ExportToVersion', 'R2012B_MDl');

  if str2double(matlabVersion) >= 8.6  
      % export to 2015a
      fprintf('\nExporting to version 2015a\n');
      save_system(modelName, 'torqueBalancingR2015aS85', 'ExportToVersion', 'R2015A_MDl');

  end 
  if str2double(matlabVersion) >= 9.1    
      % export to 2015b
      fprintf('\nExporting to version 2015b\n'); 
      save_system(modelName, 'torqueBalancingR2015bS86', 'ExportToVersion', 'R2015B_MDl');
      % export to 2016b
      fprintf('\nExporting to version 2016b\n'); 
      save_system(modelName, 'torqueBalancingR2016bS88', 'ExportToVersion', 'R2016B_MDl');
  end
  % closing the model
  close_system(modelName);
  catch ex;
end
    
fprintf('\nDone\n');

