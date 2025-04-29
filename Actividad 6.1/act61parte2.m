% Limpieza de pantalla
clear all
close all
clc

% 1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 60;             % Tiempo de simulación en segundos (s)
ts = 0.05;           % Tiempo de muestreo en segundos (s)
t = 0:ts:tf;         % Vector de tiempo
N = length(t);       % Número de muestras

% Número de trayectorias a simular (solo 3)
num_trayectorias = 3; 

for traj = 1:num_trayectorias

    % 2.1 CONDICIONES INICIALES PERSONALIZADAS
    switch traj
        case 1
            x1(1) = 0; y1(1) = 0; phi(1) = 0;
        case 2
            x1(1) = 0; y1(1) = 0; phi(1) = pi/2;
        case 3
            x1(1) = 0; y1(1) = 0; phi(1) = pi/3;
    end

    hx(1) = x1(1);   
    hy(1) = y1(1);

    % 2.2 DEFINICION DE TRAYECTORIAS Y VELOCIDADES DESEADAS
    switch traj
        case 1  % Hexágono con lado 1, centro en (0,0)
            % Hexágono: 6 segmentos, cada uno con duración igual
            L = 1; % lado
            V = 0.1; % velocidad constante (m/s)
            T_segmento = L / V;
            pasos_segmento = round(T_segmento / ts);
            total_pasos = 6 * pasos_segmento;

            if N > total_pasos
                N = total_pasos;
                t = t(1:N);
            end

            % Puntos vértices del hexágono (en sentido antihorario)
            angles = (0:5)*pi/3;
            px = cos(angles);
            py = sin(angles);

            % Crear trayectorias por tramos
            hxd = []; hyd = [];
            hxdp = []; hydp = [];
            for i = 1:6
                x_ini = px(i); y_ini = py(i);
                x_fin = px(mod(i,6)+1); y_fin = py(mod(i,6)+1);
                dx = x_fin - x_ini;
                dy = y_fin - y_ini;
                vx = dx / T_segmento;
                vy = dy / T_segmento;

                hxd = [hxd, linspace(x_ini, x_fin, pasos_segmento)];
                hyd = [hyd, linspace(y_ini, y_fin, pasos_segmento)];
                hxdp = [hxdp, vx*ones(1,pasos_segmento)];
                hydp = [hydp, vy*ones(1,pasos_segmento)];
            end

        case 2
            a = 10; 
            k = 4; 
            r = a * sin(k * t);
            hxd = r .* cos(t);
            hyd = r .* sin(t);

            dr = a * k * cos(k * t);
            hxdp = dr .* cos(t) - r .* sin(t);
            hydp = dr .* sin(t) + r .* cos(t);


        case 3
            t_scaled = linspace(0, 2*pi, length(t));  % mantener longitud t

            x_raw = 16 * sin(t_scaled).^3;
            y_raw = 13*cos(t_scaled) - 5*cos(2*t_scaled) - 2*cos(3*t_scaled) - cos(4*t_scaled);

            hxd = (15/16) * x_raw;
            hyd = (13/17) * y_raw;

            % Derivadas aproximadas numéricamente
            hxdp = [diff(hxd)/ts, 0];  % completamos último valor con 0
            hydp = [diff(hyd)/ts, 0];

    end

    % 2.3 AJUSTE DE GANANCIAS
    switch traj
        case 1
            K = [12 0; 0 12]; 
        case 2
            K = [5 0; 0 5];
        case 3
            K = [8 0; 0 20];
    end

    % 2.4 CONTROL Y SIMULACION DEL ROBOT
    for k = 1:N
        hxe(k) = hxd(k) - hx(k);
        hye(k) = hyd(k) - hy(k);
        he = [hxe(k); hye(k)];
        Error(k) = sqrt(hxe(k)^2 + hye(k)^2);

        J = [cos(phi(k)) -sin(phi(k)); sin(phi(k)) cos(phi(k))];
        qpRef = pinv(J)*( [hxdp(k); hydp(k)] + K*he );

        v(k) = qpRef(1);
        w(k) = qpRef(2);

        phi(k+1) = phi(k) + w(k)*ts;
        xp1 = v(k)*cos(phi(k)); 
        yp1 = v(k)*sin(phi(k));
        x1(k+1) = x1(k) + ts*xp1;
        y1(k+1) = y1(k) + ts*yp1;

        hx(k+1) = x1(k+1);
        hy(k+1) = y1(k+1);
    end

    % 2.5 GRAFICACION
    figure('Name',['Trayectoria ',num2str(traj)],'NumberTitle','off')
    sizeScreen = get(0,'ScreenSize'); 
    set(gcf,'position',sizeScreen); 
    
    subplot(2,2,1)
    plot(hxd,hyd,'g','lineWidth',2); hold on;
    plot(hx,hy,'r','lineWidth',2);
    title(['Trayectoria Deseada vs Realizada - Número ',num2str(traj)])
    xlabel('X [m]')
    ylabel('Y [m]')
    grid on
    legend('Deseada','Realizada')
    axis equal

    subplot(2,2,2)
    plot(t,v,'b','LineWidth',2), grid('on')
    xlabel('Tiempo [s]')
    ylabel('v [m/s]')
    title('Velocidad Lineal')

    subplot(2,2,3)
    plot(t,w,'g','LineWidth',2), grid('on')
    xlabel('Tiempo [s]')
    ylabel('w [rad/s]')
    title('Velocidad Angular')

    subplot(2,2,4)
    plot(t,Error,'r','LineWidth',2), grid('on')
    xlabel('Tiempo [s]')
    ylabel('Error [m]')
    title('Error de Posición')

    figure('Name',['Animacion Trayectoria ',num2str(traj)],'NumberTitle','off');
    scale = 4;
    MobileRobot_5;
    H1 = MobilePlot_4(x1(1), y1(1), phi(1), scale); hold on;
    H2 = plot(hx(1), hy(1), 'r', 'lineWidth',2);
    H3 = plot(hxd, hyd, 'g', 'lineWidth',2); 
    axis equal
    grid on
    for k = 1:10:N
        delete(H1);
        delete(H2);
        H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
        H2 = plot3(hx(1:k), hy(1:k), zeros(1,k), 'r', 'lineWidth',2);
        pause(ts);
    end

end
