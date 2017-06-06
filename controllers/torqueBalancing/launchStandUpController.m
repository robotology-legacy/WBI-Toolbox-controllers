clear all;
clc;

net = yarp.Network();
net.init();

port = yarp.Port();
portLog = yarp.Port();
port.open('/matlab/launchSimulink:i');
portLog.open('/matlab/logDir:i');

iterateSim = 1;

while iterateSim
    disp('Waiting on python script to tell me to start')
    btl = yarp.Bottle();
    port.read(btl);
    iterateSim = btl.get(0).asBool();
    if iterateSim 
        clearvars -EXCEPT 'iterateSim' 'port' 'portLog';
        clc;
        disp('Launching simulink controller')
        sim('torqueBalancing2016b.mdl');
        disp('Simulation finished. Waiting for save directory information...')
        btl = yarp.Bottle();
        portLog.read(btl);
        logDir = btl.get(0).asString();
        save([logDir '/wrenches'],'wrenchLeftFoot', 'wrenchRightFoot', 'wrenchLeftArm', 'wrenchRightArm');
        save([logDir '/jointData'],'jointAngles', 'jointVelocities', 'jointAccelerations');
        disp('Wrench and joint data saved to:')
        disp(logDir)
    end
end
port.close();
portLog.close();
disp('Leaving MATLAB script')
