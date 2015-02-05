function [] = drawLine(p1, p2, color, figureHandle)

theta = atan2( p2(2) - p1(2), p2(1) - p1(1));
r = sqrt( (p2(1) - p1(1))^2 + (p2(2) - p1(2))^2);

line = linspace(0,r,10);
x = p1(1) + line*cos(theta);
y = p1(2) + line*sin(theta);

figure(figureHandle);
plot(x, y, color)
drawnow;
hold on
