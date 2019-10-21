%% VISUALIZE JOINT ACCELERATIONS
clc
close all
plot_set

figureCont     = 1;
set(0,'DefaultFigureWindowStyle','Docked');

t              = ddqjLin0.time;
ddqjLinTest    = ddqjLin0.signals.values;
ddqjNonLinTest = ddqjNonLin.signals.values;
ddqjLinTest    = ddqjLinTest';
ddqjNonLinTest = ddqjNonLinTest';

for k=1:4
  
% LEFT ARM    
figure(figureCont)
subplot(2,2,k)
plot(t,ddqjLinTest(k+3,:))
hold on
plot(t,ddqjNonLinTest(k+3,:),'r')
grid on
xlabel('Time [s]')
ylabel('Joint Acc [rad/s^2]')
name = whatname('left_arm',k);
title(name)
legend('Lin Acc','NonLin Acc')

% RIGHT ARM
figure(figureCont+1)
subplot(2,2,k)
plot(t,ddqjLinTest(k+3+4,:))
hold on
plot(t,ddqjNonLinTest(k+3+4,:),'r')
grid on
xlabel('Time [s]')
ylabel('Joint Acc [rad/s^2]')
name = whatname('right_arm',k);
title(name)
legend('Lin Acc','NonLin Acc')
end

figureCont = figureCont +2;

for k=1:6

% LEFT LEG
figure(figureCont)
subplot(3,2,k)
plot(t,ddqjLinTest(k+11,:))
hold on
plot(t,ddqjNonLinTest(k+11,:),'r')
grid on
xlabel('Time [s]')
ylabel('Joint Acc [rad/s^2]')
name = whatname('left_leg',k);
title(name)
legend('Lin Acc','NonLin Acc')

% RIGHT LEG
figure(figureCont+1)
subplot(3,2,k)
plot(t,ddqjLinTest(k+11+6,:))
hold on
plot(t,ddqjNonLinTest(k+11+6,:),'r')
grid on
xlabel('Time [s]')
ylabel('Joint Acc [rad/s^2]')
name = whatname('right_leg',k);
title(name)
legend('Lin Acc','NonLin Acc')
end

figureCont  = figureCont +2;

for k=1:3
    
% TORSO
figure(figureCont)
subplot(3,1,k)
plot(t,ddqjLinTest(k,:))
hold on
plot(t,ddqjNonLinTest(k,:),'r')
grid on
xlabel('Time [s]')
ylabel('Joint Acc [rad/s^2]')
name = whatname('torso',k);
title(name)
legend('Lin Acc','NonLin Acc')
end

set(0,'DefaultFigureWindowStyle','Normal');

