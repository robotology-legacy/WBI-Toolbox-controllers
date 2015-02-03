clear all;

load('singleSupportFTSLeftAnkle.mat');
load('singleSupportWBDTLeftLeg.mat')

time     = singleSupportFTSLeftAnkle.Time;
dataFTS  = singleSupportFTSLeftAnkle.Data;
dataWBDT = singleSupportWBDTLeftLeg.Data;

normsFTS  = arrayfun(@(idx) norm(dataFTS(idx,:)), 1:size(dataFTS,1))';
normsWBDT = arrayfun(@(idx) norm(dataWBDT(idx,:)), 1:size(dataFTS,1))';


figure(2);
dataAxes = axes;

posBig = get(dataAxes, 'Position');
posSmall{1} = [0.235 0.2 0.2 0.5];
posSmall{2} = [0.61 0.3 0.2 0.5];
delete(dataAxes);

% Axes
hAxB = axes('Position', posBig);
title('huh')
hAxS1 = axes('Position', posSmall{1});
hAxS2 = axes('Position', posSmall{2});

% Plot
boxplot(hAxB,  [normsFTS, normsWBDT], 'labels', {'Measured Force from Torque Sensors - Left Single Support' 'Estimated Force from WBDT - Left Single Support'});
boxplot(hAxS1, normsFTS,  'notch', 'on', 'labels', '[zoomed] Norm of the measured force  (FTS)');
boxplot(hAxS2, normsWBDT, 'notch', 'on', 'labels', '[zoomed] Norm of the estimated force (WBDT)');

% Change axes properties
set(hAxS1, 'Color',[119/255 134/255 153/255],'XAxisLocation','top', 'YAxisLocation','left');
% set(get(hAxS1, 'XTickLabel'), {''});
set(hAxS2, 'Color',[119/255 134/255 153/255],'XAxisLocation','top', 'YAxisLocation','right');
% set(get(hAxS2, 'XTickLabel'), {''});
