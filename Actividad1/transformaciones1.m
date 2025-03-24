clear all
close all
clc

tic
% Declaración de variables simbólicas
syms x(t) y(t) th(t)  t  % Grados de Libertad del robot móvil

% Creamos el vector de posición 
xi_inercial = [x; y; th];
disp('Coordenadas generalizadas');
pretty(xi_inercial);

% Creamos el vector de velocidades
xip_inercial = diff(xi_inercial, t);
disp('Velocidades generalizadas');
pretty(xip_inercial);

% Lista de coordenadas (x, y, th)
coordenadas = [
    -5,  9,  -2;
    -3,  8,  63;
     5, -2,  90;
     0,  0,  180;
    -6,  3, -55;
    10, -2,  45;
     9,  1,  88;
     5,  2,  33;
    -1, -1,  21;
     6,  4, -40;
     5,  7,  72;
     7,  7,  30;
    11, -4,  360;
    20,  5,  270;
    10,  9,  345;
    -9, -8,  8;
     1,  1,  60;
     3,  1, -30;
    15,  2,  199;
   -10,  0,  300
];

% Inicializar lista de resultados
resultados = [];

% Iterar sobre todas las coordenadas
for i = 1:size(coordenadas, 1)
    x_i = coordenadas(i, 1);
    y_i = coordenadas(i, 2);
    th_i = coordenadas(i, 3);
    
    % Definir vector de posición y matriz de rotación
    Pos = [x_i; y_i; 0];
    Rot = [cosd(th_i) -sind(th_i) 0;
           sind(th_i)  cosd(th_i) 0;
           0          0         1];

    % Transformación del marco de referencia inercial al local
    xi_local = Rot * Pos;

    % Obtener la magnitud del vector resultante 
    magnitud = sqrt(xi_local(1)^2 + xi_local(2)^2);
    
    % Guardar resultado en la lista
    resultados = [resultados; x_i, y_i, th_i, xi_local(1), xi_local(2), magnitud];
end

% Mostrar resultados
disp('Resultados de las transformaciones:');
disp(' x      y      θ      x_local    y_local    Magnitud');
disp(resultados);

toc
