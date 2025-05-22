
% Definición del modelo de vehículo
radio = 0.1;                       % Radio de rueda [m]
eje = 0.5;                         % Distancia entre ruedas [m]
vehiculo = DifferentialDrive(radio, eje);

% Tiempos de muestreo
dt = 0.1;                          % Intervalo de muestreo [s]
tiempoSim = 0:dt:20.5;             % Vector temporal

% Posición inicial
poseIni = [2;2;0];                 % Pose inicial (x, y, theta)
trayectoria = zeros(3, numel(tiempoSim)); 
trayectoria(:,1) = poseIni;

% Cargar mapa
close all
load exampleMap

% Inicializar sensor Lidar
sensor = LidarSensor;
sensor.sensorOffset = [0, 0];
sensor.scanAngles = linspace(-pi, pi, 300);
sensor.maxRange = 1.1;

% Visualizador
visor2D = Visualizer2D;
visor2D.hasWaypoints = true;
visor2D.mapName = 'map';
attachLidarSensor(visor2D, sensor);

%% Planificación de Ruta y Control

% Puntos de referencia para la trayectoria
rutas = [poseIni(1:2)'; 
         3 6;          
         4 8;
         9 8;
         8 6;
         9 3;
         7 2];

% Controlador Pure Pursuit
ppCtrl = controllerPurePursuit;
ppCtrl.Waypoints = rutas;
ppCtrl.LookaheadDistance = 0.5;
ppCtrl.DesiredLinearVelocity = 1.0; 
ppCtrl.MaxAngularVelocity = 100;

% Controlador VFH para evitar obstáculos
vfhCtrl = controllerVFH;
vfhCtrl.DistanceLimits = [0.05 3];
vfhCtrl.NumAngularSectors = 36;
vfhCtrl.HistogramThresholds = [5 10];
vfhCtrl.RobotRadius = eje;
vfhCtrl.SafetyDistance = eje;
vfhCtrl.MinTurningRadius = 0.1;

%% Ciclo de Simulación
controlLoop = rateControl(1/dt);
for k = 2:numel(tiempoSim)
    
    % Lectura actual del sensor Lidar
    estado = trayectoria(:,k-1);
    lectura = sensor(estado);
    
    % Control de seguimiento de trayectoria y evasión
    [vLineal, wAngular, objetivo] = ppCtrl(estado);
    dirDeseada = atan2(objetivo(2)-estado(2), objetivo(1)-estado(1)) - estado(3);
    dirEvitada = vfhCtrl(lectura, sensor.scanAngles, dirDeseada);
    
    if ~isnan(dirEvitada) && abs(dirEvitada - dirDeseada) > 0.1
        wAngular = 0.5 * dirEvitada;
    end

    % Velocidad del cuerpo y actualización de posición
    velRel = [vLineal; 0; wAngular];
    velGlobal = bodyToWorld(velRel, estado);
    trayectoria(:,k) = estado + velGlobal * dt;

    % Visualización
    visor2D(trayectoria(:,k), rutas, lectura);
    waitfor(controlLoop);
end