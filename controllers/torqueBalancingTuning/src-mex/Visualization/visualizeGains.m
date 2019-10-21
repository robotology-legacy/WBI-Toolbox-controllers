%% VISUALIZE GAINS
clc
close all
plot_set

%% config parameters
set(0,'DefaultFigureWindowStyle','Docked');

KdNullVis         = KdNull.signals.values(:,:,end);
KpNullVis         = KpNull.signals.values(:,:,end);
impedancesOptVis  = impedancesOpt.signals.values(:,:,end);
dampingOptVis     = dampingOpt.signals.values(:,:,end);
KSoptVis          = KSopt.signals.values(:,:,end);
KDoptVis          = KDopt.signals.values(:,:,end);
% % KSfinVis          = KSfin.signals.values(:,:,end);
% % KDfinVis          = KDfin.signals.values(:,:,end);
KSdesVis          = KSdes.signals.values(:,:,end);
KDdesVis          = KDdes.signals.values(:,:,end);

MomGainVis        = MomGain.signals.values(:,:,end);
intMomGainVis     = intMomGain.signals.values(:,:,end);
intMomentumOptVis = intMomentumOpt.signals.values(:,:,end);
MomentumOptVis    = MomentumOpt.signals.values(:,:,end);
t                 = tout;

%% Verify the new gain matrices
% figure(1)   
% Matrix  = impedancesOptVis;
% surf(Matrix)
% title('Optimized impedances')
% 
% figure(2)   
% Matrix  = dampingOptVis;
% surf(Matrix)
% title('Optimized damping')
% 
% figure(3)   
% Matrix  = intMomentumOptVis;
% surf(Matrix)
% title('Optimized intMomentum gains')
% 
% figure(4)   
% Matrix  = MomentumOptVis;
% surf(Matrix)
% title('Optimized Momentum gains')

%% Verify the state
figure(12)
surf(KSoptVis)
title('Obtained KS')

figure(11)
surf(KDoptVis)
title('Obtained KD')

%% Verify the state
figure(5)
surf(KSdesVis)
title('Desired KS')

figure(6)
surf(KDdesVis)
title('Desired KD')

%% Verify the initial gains from Kronecher
figure(7)
surf(KdNullVis)
title('Obtained damping')

figure(8)
surf(KpNullVis)
title('Obtained impedances')

figure(9)
surf(intMomGainVis)
title('Obtained intMomentum gains')

figure(10)
surf(MomGainVis)
title('Obtained Momentum gains')


set(0,'DefaultFigureWindowStyle','Normal');

