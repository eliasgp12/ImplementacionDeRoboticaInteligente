%%PARTE 1

clear all
close all
clc

ts = 0.1;
u_val = 1.0;
w_val = 0.5;

waypoints = [-9 10;
          -7 1;
          -2 -10;
           3 1;
           5 10;
          -9 10];

x1(1) = waypoints(1,1);
y1(1) = waypoints(1,2);
phi(1) = -pi/2;

hx(1) = x1(1); hy(1) = y1(1);
u = []; w = [];

for i = 1:size(waypoints,1)-1
    ang_obj = atan2(waypoints(i+1,2)-waypoints(i,2), waypoints(i+1,1)-waypoints(i,1));
    delta_ang = wrapToPi(ang_obj - phi(end));
    t_giro = abs(delta_ang / w_val);
    N_giro = round(t_giro / ts);

    u = [u, zeros(1,N_giro)];
    w = [w, sign(delta_ang)*w_val*ones(1,N_giro)];
    phi_temp = phi(end);
    for k = 1:N_giro
        phi_temp = phi_temp + sign(delta_ang)*w_val*ts;
        phi(end+1) = phi_temp;
        x1(end+1) = x1(end);
        y1(end+1) = y1(end);
        hx(end+1) = x1(end);
        hy(end+1) = y1(end);
    end

    dist = norm(waypoints(i+1,:) - [x1(end), y1(end)]);
    t_avance = dist / u_val;
    N_avance = round(t_avance / ts);

    u = [u, u_val*ones(1,N_avance)];
    w = [w, zeros(1,N_avance)];
    for k = 1:N_avance
        phi(end+1) = phi(end);
        x1(end+1) = x1(end) + u_val*cos(phi(end))*ts;
        y1(end+1) = y1(end) + u_val*sin(phi(end))*ts;
        hx(end+1) = x1(end);
        hy(end+1) = y1(end);
    end
end

traj = 'Lazo abierto';
N = length(hx); 

figure;
set(gcf, 'Color', 'white', 'Position', get(0,'ScreenSize'));
camlight('headlight');
axis equal; grid on; box on;
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
title(traj, 'FontWeight', 'bold');
view([-0.1 90]); 


hxd = waypoints(:,1)';
hyd = waypoints(:,2)';

padding = 2;
minX = min([hx, hxd]) - padding;
maxX = max([hx, hxd]) + padding;
minY = min([hy, hyd]) - padding;
maxY = max([hy, hyd]) + padding;
axis([minX maxX minY maxY 0 1]);

scale = 5;
MobileRobot_5;
H1 = MobilePlot_4(x1(1), y1(1), phi(1), scale); hold on;
H2 = plot3(hx(1), hy(1), 0, 'r', 'LineWidth', 2); 
H3 = plot3(hxd, hyd, zeros(1,length(hxd)), 'g--o', 'LineWidth', 2); 

step = 5;
for k = 1:step:N
    delete(H1);
    delete(H2);
    H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
    H2 = plot3(hx(1:k), hy(1:k), zeros(1,k), 'r', 'LineWidth', 2);
    pause(ts);
end

t = 0:ts:(length(u)-1)*ts;
figure;
subplot(2,1,1)
plot(t,u,'b','LineWidth',2),grid on
xlabel('Tiempo'),ylabel('u [m/s]'),title('Velocidad lineal')
subplot(2,1,2)
plot(t,w,'r','LineWidth',2),grid on
xlabel('Tiempo'),ylabel('w [rad/s]'),title('Velocidad angular')

%% PARTE 2

clearvars -except MobileRobot_5 MobilePlot_4
clc

ts = 0.1;
tf = 50;
t = 0:ts:tf;
N = length(t);

waypoints = [-9 10;
          -7 1;
          -2 -10;
           3 1;
           5 10;
          -9 10];

x1(1) = waypoints(1,1);
y1(1) = waypoints(1,2);
phi(1) = -pi/2;

hx(1) = x1(1); hy(1) = y1(1);
v = zeros(1,N); w = zeros(1,N); Error = zeros(1,N);

idx_objetivo = 1;
umbral = 0.5;
num_waypoints = size(waypoints,1);

for k = 1:N
    hxd = waypoints(idx_objetivo,1);
    hyd = waypoints(idx_objetivo,2);

    hxe = hxd - hx(k);
    hye = hyd - hy(k);
    he = [hxe; hye];
    Error(k) = norm(he);

    if Error(k) < umbral && idx_objetivo < num_waypoints
        idx_objetivo = idx_objetivo + 1;
    end

    J = [cos(phi(k)) -sin(phi(k));
         sin(phi(k))  cos(phi(k))];
    K = [0.3 0; 0 0.3];

    qpRef = pinv(J) * K * he;
    v(k) = qpRef(1);
    w(k) = qpRef(2);

    phi(k+1) = phi(k) + w(k)*ts;
    x1(k+1) = x1(k) + v(k)*cos(phi(k))*ts;
    y1(k+1) = y1(k) + v(k)*sin(phi(k))*ts;
    hx(k+1) = x1(k+1);
    hy(k+1) = y1(k+1);
end

traj = 'Lazo cerrado';
scene = figure;
set(scene,'Color','white','position',get(0,'ScreenSize'));
camlight('headlight');
axis equal; grid on; box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
title(traj, 'FontWeight', 'bold');
view([-0.1 35]);
axis([-15 10 -15 15 0 1]);

hxd_full = waypoints(:,1)';
hyd_full = waypoints(:,2)';

scale = 5;
MobileRobot_5;
H1 = MobilePlot_4(x1(1), y1(1), phi(1), scale); hold on;
H2 = plot3(hx(1), hy(1), 0, 'r', 'LineWidth', 2);
H3 = plot3(hxd_full, hyd_full, zeros(1,length(hxd_full)), 'g--o', 'LineWidth', 2);

step = 5;
for k = 1:step:N
    delete(H1);
    delete(H2);
    H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
    H2 = plot3(hx(1:k), hy(1:k), zeros(1,k), 'r', 'LineWidth', 2);
    pause(ts);
end

figure;
subplot(3,1,1)
plot(t,v,'b','LineWidth',2), grid on
xlabel('Tiempo'); ylabel('v [m/s]'); title('Velocidad lineal')
subplot(3,1,2)
plot(t,w,'g','LineWidth',2), grid on
xlabel('Tiempo'); ylabel('w [rad/s]');
subplot(3,1,3)
plot(t,Error(1:N),'r','LineWidth',2), grid on
xlabel('Tiempo'); ylabel('Error de posición');

%% PARTE 3 

clearvars -except MobileRobot_5 MobilePlot_4
clc

tf = 50;
ts = 0.05;
t = 0:ts:tf;
N = length(t);

x1(1) = 0; y1(1) = 0; phi(1) = 0;
hx(1) = x1(1); hy(1) = y1(1);

waypoints = [ 0   0;
          -9  10;
          -7   1;
          -2 -10;
           3   1;
           5  10;
          -9  10];

num_segmentos = size(waypoints,1)-1;
waypoints_por_segmento = floor(N / num_segmentos);
hxd = zeros(1,N); hyd = zeros(1,N);
ti = 1;

for i = 1:num_segmentos
    if i == num_segmentos
        tf_i = N;
    else
        tf_i = ti + waypoints_por_segmento - 1;
        if tf_i > N
            tf_i = N;
        end
    end
    x_segment = linspace(waypoints(i,1), waypoints(i+1,1), tf_i - ti + 1);
    y_segment = linspace(waypoints(i,2), waypoints(i+1,2), tf_i - ti + 1);
    hxd(ti:tf_i) = x_segment;
    hyd(ti:tf_i) = y_segment;
    ti = tf_i + 1;
end

hxdp = [diff(hxd)/ts, 0];
hydp = [diff(hyd)/ts, 0];

for k = 1:N
    hxe(k) = hxd(k) - hx(k);
    hye(k) = hyd(k) - hy(k);
    he = [hxe(k); hye(k)];
    Error(k) = norm(he);

    J = [cos(phi(k)) -sin(phi(k));
         sin(phi(k))  cos(phi(k))];
    K = [10 0; 0 10];

    hdp = [hxdp(k); hydp(k)];
    qpRef = pinv(J)*(hdp + K*he*tanh(norm(he)));
    v(k) = max(min(qpRef(1), 1.5), -1.5);
    w(k) = max(min(qpRef(2), 1.5), -1.5);

    if k > N-20 && Error(k) < 0.3
        v(k) = 0;
        w(k) = 0;
    end

    phi(k+1) = phi(k) + w(k)*ts;
    x1(k+1) = x1(k) + ts*v(k)*cos(phi(k));
    y1(k+1) = y1(k) + ts*v(k)*sin(phi(k));
    hx(k+1) = x1(k+1);
    hy(k+1) = y1(k+1);
end

scene = figure;
set(scene,'Color','white','Position',get(0,'ScreenSize'));
camlight('headlight'); axis equal; grid on; box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
title('Lazo cerrado con posiciones y velocidades personalizadas');
view([-0.1 90]); axis([-15 10 -15 15 0 1]);

scale = 4;
MobileRobot_5;
H1 = MobilePlot_4(x1(1), y1(1), phi(1), scale); hold on;
H2 = plot3(hx(1), hy(1), 0, 'r', 'LineWidth', 2);
H3 = plot3(hxd, hyd, zeros(1,N), 'g--', 'LineWidth', 2);

step = 5;
for k = 1:step:N
    delete(H1); delete(H2);
    H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
    H2 = plot3(hx(1:k), hy(1:k), zeros(1,k), 'r', 'LineWidth', 2);
    pause(ts);
end

figure;
subplot(3,1,1)
plot(t,v,'b','LineWidth',2), grid on
xlabel('Tiempo'); ylabel('v [m/s]'); title('Velocidad lineal')
subplot(3,1,2)
plot(t,w,'g','LineWidth',2), grid on
xlabel('Tiempo'); ylabel('w [rad/s]');
subplot(3,1,3)
plot(t,Error(1:N),'r','LineWidth',2), grid on
xlabel('Tiempo'); ylabel('Error de posición');