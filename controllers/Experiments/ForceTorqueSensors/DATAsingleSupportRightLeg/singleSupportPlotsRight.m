clear all;

load('singleSupportFTSRightAnkle.mat')
load('singleSupportWBDTRightLeg.mat')

time     = singleSupportFTSRightAnkle.Time;
dataFTS  = singleSupportFTSRightAnkle.Data;
dataWBDT = singleSupportWBDTRightLeg.Data;

% Removing sensor offset
rot = [1 0 0; 0 -1 0; 0 0 -1];
leftFootOffset = rot*[-0.734 -4.066 -3.9819]';
rightFootOffset= rot*[86.670 -31.390 10.3837]';

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
boxplot(hAxB,  [normsFTS, normsWBDT], 'labels', {'Measured Force from Torque Sensors - Right Single Support' 'Estimated Force from WBDT - Right Single Support'});
boxplot(hAxS1, normsFTS,  'notch', 'on', 'labels', '[zoomed] Norm of the measured force  (FTS)');
boxplot(hAxS2, normsWBDT, 'notch', 'on', 'labels', '[zoomed] Norm of the estimated force (WBDT)');

% Change axes properties
set(hAxS1, 'Color',[119/255 134/255 153/255],'XAxisLocation','top', 'YAxisLocation','left');
% set(get(hAxS1, 'XTickLabel'), {''});
set(hAxS2, 'Color',[119/255 134/255 153/255],'XAxisLocation','top', 'YAxisLocation','right');
% set(get(hAxS2, 'XTickLabel'), {''});
