clearvars
close all
clc

% Construcción de las transformaciones homogéneas 
T_base = SE3;  % Trama de referencia inicial

T1 = SE3(rotz(2*pi), [0 0 0]);        % Rotación completa sobre Z
T2 = SE3(rotx(-pi/2), [0 0 0]);       % Rotación negativa sobre X
T3 = SE3(rotz(2*pi), [3 0 0]);        % Rotación completa sobre Z con desplazamiento en X
T4 = SE3(rotz(-pi/2), [3 0 0]);       % Rotación negativa sobre Z con el mismo desplazamiento
T5 = SE3(rotx(-pi/2), [0 0 0]);       % Segunda rotación sobre X
T6 = SE3(rotz(2*pi), [0 0 0]);        % Última rotación sobre Z

% Composición de las transformaciones
T_2 = T1 * T2;
T_3 = T_2 * T3;
T_4 = T_3 * T4;
T_5 = T_4 * T5;
T_6 = T_5 * T6;

% Coordenadas para visualización de estructura 
x_line = [0 6];
y_line = [0 0];
z_line = [0 0];

% Gráfica de estructura y tramas 
plot3(x_line, y_line, z_line, 'LineWidth', 1.5);
axis([-1 8 -3 3 -1 5]);
grid on
hold on

% Trama global inicial
trplot(T_base, 'rgb', 'axis', [-1 8 -3 3 -1 5]);

% Animación de cada transformación paso a paso 
pause;
tranimate(T_base, T1, 'rgb', 'axis', [-1 8 -3 3 -1 5]);

pause;
tranimate(T1, T_2, 'rgb', 'axis', [-1 8 -3 3 -1 5]);

pause;
tranimate(T_2, T_3, 'rgb', 'axis', [-1 8 -3 3 -1 5]);

pause;
tranimate(T_3, T_4, 'rgb', 'axis', [-1 8 -3 3 -1 5]);

pause;
tranimate(T_4, T_5, 'rgb', 'axis', [-1 8 -3 3 -1 5]);

pause;
tranimate(T_5, T_6, 'rgb', 'axis', [-1 8 -3 3 -1 5]);
