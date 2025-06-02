clearvars
close all
clc

%VARIABLES SIMBÓLICAS
% Definición de variables simbólicas dependientes del tiempo
syms q1(t) q2(t) q3(t) q4(t) q5(t) q6(t) q7(t) t lA lB lC

jointType = [0 0 0 0 0 0 0];

qVec = [-q1, q2, q3, q4, q5, q6, q7];

qDot = diff(qVec, t);

% Número de grados de libertad
dof = length(jointType);

%MATRICES DE ROTACIÓN DE CADA ARTICULACIÓN
% Cada articulación rotacional se representa con una matriz de rotación en Z
Rz1 = [cos(-q1) -sin(-q1) 0; sin(-q1) cos(-q1) 0; 0 0 1];
Rz2 = [cos(q2) -sin(q2) 0; sin(q2) cos(q2) 0; 0 0 1];
Rz3 = [cos(q3) -sin(q3) 0; sin(q3) cos(q3) 0; 0 0 1];
Rz4 = [cos(q4) -sin(q4) 0; sin(q4) cos(q4) 0; 0 0 1];
Rz5 = [cos(q5) -sin(q5) 0; sin(q5) cos(q5) 0; 0 0 1];
Rz6 = [cos(q6) -sin(q6) 0; sin(q6) cos(q6) 0; 0 0 1];
Rz7 = [cos(q7) -sin(q7) 0; sin(q7) cos(q7) 0; 0 0 1];

%ROTACIONES CONSTANTES ENTRE MARCOS
Ry90 = [0 0 1; 0 1 0; -1 0 0];
Rym90 = [0 0 -1; 0 1 0; 1 0 0];
Rz90 = [0 -1 0; 1 0 0; 0 0 1];
Rzm90 = [0 1 0; -1 0 0; 0 0 1];

%DEFINICIÓN DE POSICIONES Y ROTACIONES
P = sym(zeros(3, 1, dof));    
R = sym(zeros(3, 3, dof));    

P(:,:,1) = [0; 0; 0];           R(:,:,1) = Rz1;
P(:,:,2) = [0; 0; 0];           R(:,:,2) = Ry90 * Rzm90 * Rz2;
P(:,:,3) = [0; 0; 0];           R(:,:,3) = Rym90 * Rzm90 * Rz3;
P(:,:,4) = [lA; 0; 0];          R(:,:,4) = Rz4;
P(:,:,5) = [lB; 0; 0];          R(:,:,5) = Ry90 * Rz5;
P(:,:,6) = [0; 0; 0];           R(:,:,6) = Ry90 * Rz90 * Rz6;
P(:,:,7) = [lC; 0; 0];          R(:,:,7) = Rz7;

%MATRICES DE TRANSFORMACIÓN HOMOGÉNEA
% Se inicializan matrices de transformación homogénea locales y globales
zero_row = zeros(1, 3); 
A = sym(zeros(4, 4, dof));
T = sym(zeros(4, 4, dof));  
PO = sym(zeros(3, 1, dof)); 
RO = sym(zeros(3, 3, dof));

% Cálculo de las matrices A y T para cada articulación
for i = 1:dof
    A(:,:,i) = simplify([R(:,:,i), P(:,:,i); zero_row, 1]); % Transformación local
    if i == 1
        T(:,:,i) = A(:,:,i); % La primera transformación es igual a la local
    else
        T(:,:,i) = simplify(T(:,:,i-1) * A(:,:,i)); % Composición acumulativa
    end
    % Mostrar matriz de transformación global
    disp(['Matriz de Transformación Global T' num2str(i)]);
    pretty(T(:,:,i))

    % Almacenar rotación y posición global del marco
    RO(:,:,i) = T(1:3,1:3,i);
    PO(:,:,i) = T(1:3,4,i);
end

%CÁLCULO DEL JACOBIANO ANALÍTICO
% Inicialización del Jacobiano lineal y angular
Jv = sym(zeros(3, dof));  % Jacobiano lineal
Jw = sym(zeros(3, dof));  % Jacobiano angular

for j = 1:dof
    if jointType(j) == 0
        % Articulación rotacional
        try         
            Jv(:,j) = cross(RO(:,3,j-1), PO(:,:,dof) - PO(:,:,j-1));
            Jw(:,j) = RO(:,3,j-1); 
        catch           
            Jv(:,j) = cross([0 0 1], PO(:,:,dof));
            Jw(:,j) = [0 0 1];
        end
    else
        % Articulación prismática
        try
            Jv(:,j) = RO(:,3,j-1);
        catch
            Jv(:,j) = [0 0 1];
        end
        Jw(:,j) = [0 0 0];
    end
end

% Simplificación y visualización del Jacobiano
Jv = simplify(Jv);
Jw = simplify(Jw);
disp('Jacobiano lineal analítico:');
pretty(Jv);
disp('Jacobiano angular analítico:');
pretty(Jw);

%CÁLCULO DE VELOCIDADES LINEAL Y ANGULAR
disp('Velocidad lineal V = Jv * qDot');
V = simplify(Jv * qDot');
pretty(V);

disp('Velocidad angular W = Jw * qDot');
W = simplify(Jw * qDot');
pretty(W);
