clear all;
tic; 
balancingController_QP([],[],[],'compile'); 
compileTime = toc() 
balancingController_QP([],[],[],'term');

tic;
sim('balancingController_QP.mdl');
executionTime = toc()

realTimeFactor = simulationTime/(executionTime-compileTime)