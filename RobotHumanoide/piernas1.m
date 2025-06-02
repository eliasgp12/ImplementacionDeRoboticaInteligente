clearvars
close all
clc

%Definición de transformaciones
BaseFrame = SE3; 
T1 = SE3(rotx(pi/2), [0 0 3]);   
T2 = SE3([2 0 0]);            
T3 = SE3([2 0 0]);            
T4 = SE3([2 0 0]);               

%Transformaciones compuestas
Frame2 = T1 * T2;
Frame3 = Frame2 * T3; 
Frame4 = Frame3 * T4;    

% Coordenadas para graficar 
x_coords = [0 0 6];
y_coords = [0 0 0];
z_coords = [0 3 3];

% Visualización de la estructura 
plot3(x_coords, y_coords, z_coords, 'LineWidth', 1.5);
axis([-1 8 -1 8 -1 4]);
grid on
hold on

% Trama base
trplot(BaseFrame, 'rgb', 'axis', [-1 8 -1 8 -1 4]);

% Animación de cada transformación 
pause;
tranimate(BaseFrame, T1, 'rgb', 'axis', [-1 8 -1 8 -1 4]);

pause;
tranimate(T1, Frame2, 'rgb', 'axis', [-1 8 -1 8 -1 4]);

pause;
tranimate(Frame2, Frame3, 'rgb', 'axis', [-1 8 -1 8 -1 4]);

pause;
tranimate(Frame3, Frame4, 'rgb', 'axis', [-1 8 -1 8 -1 4]);

  