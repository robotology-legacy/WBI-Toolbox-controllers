clear;

comInput = dlmread('comTraj.txt');
qDesInput = dlmread('posturalTraj.txt');
constraintsInput = dlmread('constraints.txt');

Ts = 0.01;

dataLength = length(comInput);
assert(dataLength == length(qDesInput) && dataLength == length(constraintsInput), 'Vectors must be of the same size');

com = struct;
com.time = 0:Ts:(dataLength - 1)* Ts;
com.signals.dimensions = 9;
com.signals.values = comInput;

qDes = struct;
qDes.time = com.time;
qDes.signals.dimensions = size(qDesInput, 2);
qDes.signals.values = qDesInput;

constraints = struct;
constraints.time = com.time;
constraints.signals.dimensions = 2;
constraints.signals.values = constraintsInput;

controllerName = 'matlabTorqueBalancing';
comPort = ['/' controllerName '/comDes:i'];
constraintsPort = ['/' controllerName '/constraints:i'];
qDesPort = ['/' controllerName '/qDes:i'];

clear comInput qDesInput constraintsInput