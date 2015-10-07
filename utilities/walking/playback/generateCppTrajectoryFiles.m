function generateCppTrajectoryFiles()
%%Additional definitions
varname=@(x) inputname(1);

%%Load parts
filename = 'icp-insideJointLimits-smallCoMOscillations';
extension = '.csv';
[~, parts] = TrajectoryLoader([pwd,filesep,filename, extension], 1);

%%write parts to space delimited files
%retrieve auxiliary struct
if (~isKey(parts, 'auxiliary'))
    disp('Error: parts structure is malformed');
    return;
end

auxiliary = parts('auxiliary');
parts.remove('auxiliary');

for key = parts.keys
   data = [auxiliary, parts(key{1})];
   
   partFile = [pwd,filesep,filename,'_',key{1},'.txt'];
   save(partFile, varname(data), '-ascii');
end

end