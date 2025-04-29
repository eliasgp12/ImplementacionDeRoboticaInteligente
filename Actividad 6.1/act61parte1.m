% Limpieza de pantalla
clear all
close all
clc

% 1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 40;             % Tiempo de simulación en segundos (s)
ts = 0.05;           % Tiempo de muestreo en segundos (s)
t = 0:ts:tf;         % Vector de tiempo
N = length(t);       % Número de muestras

% Número de trayectorias a simular
num_trayectorias = 8; 

for traj = 1:num_trayectorias

    % 2.1 CONDICIONES INICIALES PERSONALIZADAS
    switch traj
        case 1
            x1(1) = 0; y1(1) = 0; phi(1) = pi/2;
        case 2
            x1(1) = 0; y1(1) = 0; phi(1) = pi/2;
        case 3
            x1(1) = 0; y1(1) = 0; phi(1) = pi/3;
        case 4
            x1(1) = 0; y1(1) = 0; phi(1) = pi/4;
        case 5
            x1(1) = 0; y1(1) = 0; phi(1) = pi/3;
        case 6
            x1(1) = 0; y1(1) = 0; phi(1) = pi/2;
        case 7
            x1(1) = 0; y1(1) = 0; phi(1) = 0;
        case 8
            x1(1) = 0; y1(1) = 0; phi(1) = 7*pi/4;
    end

    hx(1) = x1(1);   
    hy(1) = y1(1);

    % 2.2 DEFINICION DE TRAYECTORIAS Y VELOCIDADES DESEADAS
    switch traj
        case 1
            hxd = 2*cos(0.2*t);
            hyd = 2*sin(0.4*t);
            hxdp = -0.1*sin(0.2*t);
            hydp = 0.2*cos(0.4*t);
        case 2
            hxd = t - 3*sin(t);
            hyd = 4 - 3*cos(t);
            hxdp = 0.5*(1 - 3*cos(t));
            hydp = 0.5*(3*sin(t));
        case 3
            hxd = 3*cos(t) - cos(3*t);
            hyd = 4*sin(3*t);
            hxdp = -0.1*(sin(t) - 3*sin(t));
            hydp = 0.5*(3*cos(3*t));
        case 4
            hxd = cos(t) + 0.5*cos(7*t) + (1/3)*sin(17*t);
            hyd = sin(t) + 0.5*sin(7*t) + (1/3)*cos(17*t);
            hxdp = -0.4*(sin(t) + 3.5*sin(7*t) - (17/3)*cos(17*t));
            hydp = 0.4*(cos(t) + 3.5*cos(7*t) - (17/3)*sin(17*t));
        case 5
            hxd = 17*cos(t) + 7*cos(17+7*t);
            hyd = 17*sin(t) - 7*sin(17+7*t);
            hxdp = -0.5*(17*sin(t) + 49*sin(17+7*t));
            hydp = 0.5*(17*cos(t) - 49*cos(17+7*t));
        case 6
            hxd = 2*cos(t);
            hyd = 2*sin(t);
            hxdp = -0.2*sin(t);
            hydp = 0.2*cos(t);
        case 7
            hxd = 5*t - 4*sin(t);
            hyd = 5*t - 4*cos(t);
            hxdp = 0.5*(5 - 4*cos(t));
            hydp = 0.5*(5 + 4*sin(t));
        case 8
            hxd = 4*cos(t) + cos(4*t);
            hyd = 4*sin(t) - sin(4*t);
            hxdp = -0.2*(sin(t) + 4*sin(4*t));
            hydp = 0.2*(cos(t) - 4*cos(4*t));
    end

    % 2.3 AJUSTE DE GANANCIAS
    switch traj
        case 1
            K = [15 0; 0 20];
        case 2
            K = [13 0; 0 13];
        case 3
            K = [8 0; 0 20];
        case 4
            K = [20 0; 0 30];
        case 5
            K = [13 0; 0 13];
        case 6
            K = [17 0; 0 17];
        case 7
            K = [13 0; 0 13];
        case 8
            K = [13 0; 0 13];
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
