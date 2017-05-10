%% EXPORTMODELS 
%  exports models in previous Simulink versions for keeping back-compatibility. 
% 
%  - available Matlab versions: 2016b, 2015b, 2015a, 2012b
%
%  - this script should be launched from a Matlab version greater than 2012b
%    (either 2015a, 2015b, 2016b)
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

fprintf('\n### Exporting Simulink model to previous versions\n');
fprintf('\n### Starting Matlab version must be one of the following: 2015a, 2015b, 2016b\n');
fprintf('\n### Exported versions: 2015b, 2015a, 2012b\n');
fprintf('\n### If you need to export other versions, just modify this script accordingly.\n');

% at the moment, this script can export models only from a Matlab version 
% greater than 2015a
if (verLessThan('matlab', '8.5'))
    error('This script should be launched with a MATLAB version >= than 2015a');
    quit; %#ok<UNRCH>
end

% find the current matlab version and prepare the environment accordingly
versionInfo   = version;
matlabVersion = versionInfo(1:3);

if strcmp(matlabVersion,'9.1')

    versionName = '2016b';

elseif strcmp(matlabVersion,'8.6')
    
    versionName = '2015b'; 
   
elseif strcmp(matlabVersion,'8.5')
    
    versionName = '2015a';
    
end
    
modelName = ['torqueBalancing' versionName];

%% Load the model 
try
  open_system(modelName,'loadonly');
  fprintf('\nModel loaded\n');
  fprintf(['\nCurrent Matlab version: ' versionName '\n']);

  % export to 2012b
  fprintf('\nExporting to version 2012b\n');
  save_system(modelName, 'torqueBalancingR2012b', 'ExportToVersion', 'R2012B_MDl');

  if strcmp(matlabVersion,'9.1') || strcmp(matlabVersion,'8.6')   
      % export to 2015a
      fprintf('\nExporting to version 2015a\n');
      save_system(modelName, 'torqueBalancing2015a', 'ExportToVersion', 'R2015A_MDl');

  end 
   
  if strcmp(matlabVersion,'9.1')    
      % export to 2015b
      fprintf('\nExporting to version 2015b\n'); 
      save_system(modelName, 'torqueBalancing2015b', 'ExportToVersion', 'R2015B_MDl');
  end
  
  % closing the model
  close_system(modelName);
  catch ex;

end
    
fprintf('\nDone\n');

