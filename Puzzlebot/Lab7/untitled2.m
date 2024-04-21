clear all;
clc;
close all;

bagReader = ros2bagreader("cuadradosinEsq/");
baginfo = ros2("bag","info","cuadradosinEsq/");

msgs = readMessages(bagReader);

bagSelL = select(bagReader,"Topic", "/positionX");
bagSelR = select(bagReader,"Topic", "/positionY");

msgsFilteredL = readMessages(bagSelL);
msgsFilteredR = readMessages(bagSelR);

msgsFilteredL{1};
msgsFilteredR{1};

xPoints = cellfun(@(m) double(m.data),msgsFilteredL);
yPoints = cellfun(@(m) double(m.data),msgsFilteredR);

%RECTA
%waypointsX = [0.00, 0.25, 0.50, 0.75, 1.00, 1.25, 1.50, 1.75, 2.00];
%waypointsY = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00];

%CARRIL IZQUIERDO
%waypointsX = [0.00, 0.25, 0.50, 0.75, 1.00, 1.00, 1.00, 1.25, 1.50, 1.75, 2.00];
%waypointsY = [0.00, 0.00, 0.00, 0.00, 0.00, 0.25, 0.50, 0.50, 0.50, 0.50, 0.50];

%CARRIL DERECHO
%waypointsX = [0.00, 0.25, 0.50, 0.75, 1.00, 1.00, 1.00, 1.25, 1.50, 1.75, 2.00];
%waypointsY = [0.00, 0.00, 0.00, 0.00, 0.00, -0.25, -0.50, -0.50, -0.50, -0.50, -0.50];

%CUADRADO
waypointsX = [0.00, 0.25, 0.50, 0.75, 1.00, 1.20, 1.30, 1.50, 1.50, 1.50, 1.50, 1.50, 1.50, 1.30, 1.20, 1.00, 0.75, 0.50, 0.20, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00];
waypointsY = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, -0.25, -0.50, -0.75, -1.00, -1.20, -1.30, -1.50, -1.50, -1.50, -1.50, -1.50, -1.50, -1.25, -1.00, -0.75, -0.50, -0.25, 0.00];
%TRIANGULO
%waypointsX = [0.00, 0.25, 0.50, 0.75, 1.00, 0.84, 0.67, 0.50, 0.30, 0.17, 0.00];
%waypointsY = [0.00, 0.00, 0.00, 0.00, 0.00, 0.30, 0.60, 1.00, 0.60, 0.30, 0.00];
%%
plot(xPoints, yPoints, 'LineWidth', 2)
grid on
xlabel('X[m]')
ylabel('Y[m]')
title('Trayectoria del triangulo')

hold on
plot(waypointsX, waypointsY, 'o-');
hold off
axis equal
