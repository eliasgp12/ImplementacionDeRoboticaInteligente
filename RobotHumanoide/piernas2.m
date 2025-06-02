clearvars
close all
clc

% Construcción de transformaciones homogéneas 
Frame0 = SE3;  % Sistema de referencia base

% Transformaciones individuales
Step1 = SE3(rotz(pi), [3 0 0]);           
Step2 = SE3(roty(pi/2), [0 0 0]);                 
Step3 = SE3(rotx(deg2rad(150)), [-2 0 0]);           

% Encadenamiento de transformaciones
Frame2 = Step1 * Step2;
Frame3 = Frame2 * Step3;  

% Puntos de referencia para visualización 
x_vals = [0 3 3 0 0 0     0     0 0     3];
y_vals = [0 0 0 0 0 5.196 5.196 0 5.196 0];
z_vals = [0 0 2 2 0 0     2     2 2     2];

% Gráfica de la estructura 
plot3(x_vals, y_vals, z_vals, 'LineWidth', 1.5);
axis([-1 4 -1 6 -1 2]);
grid on
hold on

% Visualización de tramas con animación 
% Trama base
trplot(Frame0, 'rgb', 'axis', [-1 4 -1 6 -1 2]);

% Animación paso a paso
pause;
tranimate(Frame0, Step1, 'rgb', 'axis', [-1 4 -1 6 -1 2]);

pause;
tranimate(Step1, Frame2, 'rgb', 'axis', [-1 4 -1 6 -1 2]);

pause;
tranimate(Frame2, Frame3, 'rgb', 'axis', [-1 4 -1 6 -1 2]);
