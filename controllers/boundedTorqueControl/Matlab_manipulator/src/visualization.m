%Visualization of the 2-link robot

%Plot axes
ax = axes('XLim',[-3 4],'YLim',[-3.5 3.5],'ZLim',[-1.5 1.5]);

%Define objects
%link1
h(1) = rectangle('Position', [-0.04 0 0.08 model.a1], 'Curvature',1, ...
    'FaceColor','g','EdgeColor','k','LineWidth',1); 
%link2
h(2) = rectangle('Position', [-0.04 -model.a1 0.08 model.a2], 'Curvature',1, ...
    'FaceColor','b','EdgeColor','k','LineWidth',1);
%link2 reference
h(3) = rectangle('Position', [-0.04 -model.a1 0.08 model.a2], 'Curvature',1, ...
    'FaceColor','none','EdgeColor','r','LineWidth',1); 
%joint1
h(4) = rectangle('Position',[-0.02 -0.02 0.04 0.04],'Curvature',[1 1], ...
    'FaceColor', 'k', 'EdgeColor', 'k', 'LineWidth', 1);
%joint2
h(5) = rectangle('Position',[-0.02 -0.02 0.04 0.04],'Curvature',[1 1], ...
    'FaceColor', 'r', 'EdgeColor', 'r', 'LineWidth', 1);
%link1 reference
h(6) = rectangle('Position', [-0.04 0 0.08 model.a1], 'Curvature',1, ...
    'FaceColor','none','EdgeColor','r','LineWidth',1); 

link1 = hgtransform('Parent', ax);
link2 = hgtransform('Parent', ax);
joint1 = hgtransform('Parent', ax);
joint2 = hgtransform('Parent', ax);
referenceLink2 = hgtransform('Parent', ax);
referenceLink1 = hgtransform('Parent', ax);

set(h(1), 'Parent', link1);
set(h(2), 'Parent', link2);
set(h(3), 'Parent', referenceLink2);
set(h(4), 'Parent', joint1);
set(h(5), 'Parent', joint2);
set(h(6), 'Parent', referenceLink1);

%position objects in the plot
q1 = yout.signals.values(1,1);
Txy1 = makehgtform('zrotate', q1 - pi/2); %transform matrix
set(link1, 'Matrix', Txy1);

Txy1r = makehgtform('zrotate', r(1) - pi/2); %transform matrix
set(referenceLink1, 'Matrix', Txy1r);

q2 = yout.signals.values(1,2);
Txy2 = makehgtform('translate',  [model.a1*cos(q1) model.a1*sin(q1) 0], 'zrotate', q1 + q2 + pi/2);
set(joint2, 'Matrix', Txy2);
set(link2, 'Matrix', Txy2);

Txy2r = makehgtform('translate',  [model.a1*cos(q1) model.a1*sin(q1) 0], 'zrotate', q1 + r(2) + pi/2); 
set(referenceLink2, 'Matrix', Txy2r);


%% Draw a legend (what is actuated and not, what is the reference and not)
%link1
legend(1) = rectangle('Position', [1 3 model.a1 0.08], 'Curvature',1, ...
    'FaceColor','g','EdgeColor','k','LineWidth',1); 
%link2
legend(2) = rectangle('Position', [1 3-0.2 model.a2 0.08], 'Curvature',1, ...
    'FaceColor','b','EdgeColor','k','LineWidth',1); 
%link2 reference
legend(3) = rectangle('Position', [1 3-0.4 model.a2 0.08], 'Curvature',1, ...
    'FaceColor','none','EdgeColor','r','LineWidth',1); 
%joint1
legend(4) = rectangle('Position',[1+model.a1-0.02 3-0.6 0.04 0.04],'Curvature',[1 1], ...
    'FaceColor', 'k', 'EdgeColor', 'k', 'LineWidth', 1); 
%joint2
legend(5) = rectangle('Position',[1+model.a2-0.02 3-0.8 0.04 0.04],'Curvature',[1 1], ...
    'FaceColor', 'r', 'EdgeColor', 'r', 'LineWidth', 1); 

legendlink1 = hgtransform('Parent', ax);
legendlink2 = hgtransform('Parent', ax);
legendjoint1 = hgtransform('Parent', ax);
legendjoint2 = hgtransform('Parent', ax);
legendreferenceLink2 = hgtransform('Parent', ax);
set(legend(1), 'Parent', legendlink1);
set(legend(2), 'Parent', legendlink2);
set(legend(3), 'Parent', legendreferenceLink2);
set(legend(4), 'Parent', legendjoint1);
set(legend(5), 'Parent', legendjoint2);

text(1+model.a1+0.1,3.05,'Link1')
text(1+model.a1+0.1,3.05-0.2,'Link2')
text(1+model.a1+0.1,3.05-0.4,'Link reference')
text(1+model.a1+0.1,3.05-0.6,'Joint1, actuated')
text(1+model.a1+0.1,3.05-0.8,'Joint2, actuated')

%% Animation
waitforbuttonpress;
set(gcf,'CurrentCharacter', '@'); % set to a dummy character

for i = 1:length(yout.time)
    
    %Move link1 and link2 as obtained by the simulation
    q1 = yout.signals.values(i,1);
    Txy1 = makehgtform('zrotate', q1 - pi/2);
    set(link1,'Matrix',Txy1)
    
    q2 = yout.signals.values(i,2);
    Txy2 = makehgtform('translate', [model.a1*cos(q1) model.a1*sin(q1) 0], 'zrotate', q1 + q2 + pi/2);
    set(link2,'Matrix',Txy2)
    set(joint2, 'Matrix', Txy2);
    
    
    r1 = yout.signals.values(i,5);
    Txyr1 = makehgtform('zrotate', r1 - pi/2);
    set(referenceLink1,'Matrix',Txyr1)
    
    r2 = yout.signals.values(i,6); %rAmplitude * (1 + sin(rFrequency * yout.time(i))) - pi/2;
    Txyr = makehgtform('translate', [model.a1*cos(q1) model.a1*sin(q1) 0], 'zrotate',  q1 + r2 + pi/2);
    set(referenceLink2,'Matrix',Txyr)
    
    drawnow
    
    if i < length(yout.time)
        pause((yout.time(i+1) - yout.time(i)))
    end
 
    %stops the function when pressing 'q', closes the figure
    if strcmp(get(gcf,'CurrentKey'), 'q')
        if i > 1
            close all;
            break;
        end
    end
    
    %pauses the function when pressing other keys
    if get(gcf,'CurrentCharacter') ~= '@' % has it changed from the dummy character?
        waitforbuttonpress;
        set(gcf,'CurrentCharacter', '@'); % set to a dummy character
    end

end

