%% VISUALIZE JOINT POSITIONS
clc
close all

%% initialization
plot_set
set(0,'DefaultFigureWindowStyle','Docked');
figureCont     = 1;

t              = qj.time;
tAss           = (5./sqrt(diag(gain.KSdes)))+gain.tStep;

for k = 1:length(tAss)    
    index(k)   = sum(t>tAss(k))+1;
end

qjReal    = qj.signals.values;
qjDesired = qjDes.signals.values;
qjReal    = 180/pi*qjReal';
qjDesired = 180/pi*qjDesired';
STEP      = qjDesired(:,end)-qjDesired(:,1);

%% joint positions
for k=1:4
  
% LEFT ARM    
figure(figureCont)
subplot(2,2,k)
plot(t,qjReal(k+3,:))
hold on
plot(t,qjDesired(k+3,:),'r')
plot(tAss(k+3),qjReal(k+3,end-index(k+3)),'ok')
plot(t, qjDesired(k+3,:)+0.05*STEP(k+3))
plot(t, qjDesired(k+3,:)-0.05*STEP(k+3))
grid on
xlabel('Time [s]')
ylabel('Joint Pos [deg]')
name = whatname('left_arm',k);
title(name)
legend('Joint pos','Joint ref')

% RIGHT ARM
figure(figureCont+1)
subplot(2,2,k)
plot(t,qjReal(k+3+4,:))
hold on
plot(t,qjDesired(k+3+4,:),'r')
plot(tAss(k+3+4),qjReal(k+3+4,end-index(k+3+4)),'ok')
plot(t, qjDesired(k+3+4,:)+0.05*STEP(k+3+4))
plot(t, qjDesired(k+3+4,:)-0.05*STEP(k+3+4))
grid on
xlabel('Time [s]')
ylabel('Joint Pos [deg]')
name = whatname('right_arm',k);
title(name)
legend('Joint pos','Joint ref')
end

figureCont = figureCont +2;

for k=1:6

% LEFT LEG
figure(figureCont)
subplot(3,2,k)
plot(t,qjReal(k+11,:))
hold on
plot(t,qjDesired(k+11,:),'r')

grid on
xlabel('Time [s]')
ylabel('Joint Pos [deg]')
name = whatname('left_leg',k);
title(name)
legend('Joint pos','Joint ref')

% RIGHT LEG
figure(figureCont+1)
subplot(3,2,k)
plot(t,qjReal(k+11+6,:))
hold on
plot(t,qjDesired(k+11+6,:),'r')

grid on
xlabel('Time [s]')
ylabel('Joint Pos [deg]')
name = whatname('right_leg',k);
title(name)
legend('Joint pos','Joint ref')
end

figureCont  = figureCont +2;

for k=1:3
    
% TORSO
figure(figureCont)
subplot(3,1,k)
plot(t,qjReal(k,:))
hold on
plot(t,qjDesired(k,:),'r')
plot(tAss(k),qjReal(k,end-index(k)),'ok')
plot(t, qjDesired(k,:)+0.05*STEP(k))
plot(t, qjDesired(k,:)-0.05*STEP(k))
grid on
xlabel('Time [s]')
ylabel('Joint Pos [deg]')
name = whatname('torso',k);
title(name)
legend('Joint pos','Joint ref')
end

set(0,'DefaultFigureWindowStyle','Normal');

