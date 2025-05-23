
% Definir parámetros del robot
radioRueda = 0.1;           % Radio de la rueda [m]
distEjes = 0.5;             % Distancia entre ruedas [m]
robot = DifferentialDrive(radioRueda, distEjes);

% Tiempos de simulación
Ts = 0.1;                                  % Tiempo de muestreo [s]
tiempo = 0:Ts:10.5;                        % Vector de tiempo

% Posición inicial del robot
poseInicial = [4; 2; pi/2];                % [x, y, theta]
trayectoria = zeros(3, numel(tiempo));    % Matriz de posiciones
trayectoria(:,1) = poseInicial;

% Cargar mapa
close all
load exampleMap

% Crear sensor lidar
sensorLidar = LidarSensor;
sensorLidar.sensorOffset = [0, 0];
sensorLidar.scanAngles = linspace(0, 2*pi, 300); 
sensorLidar.maxRange = 0.36;

% Crear visualizador
visor = Visualizer2D;
visor.hasWaypoints = true;
visor.mapName = 'map';
attachLidarSensor(visor, sensorLidar);

%% Planeación y seguimiento de trayectorias

% ----- Trayectoria de referencia --------
puntosRuta = [poseInicial(1:2)'; 
              4 2;
              4 6;
              9 6;
              9 2];

% Controlador Pure Pursuit
ppController = controllerPurePursuit;
ppController.Waypoints = puntosRuta;
ppController.LookaheadDistance = 0.4;
ppController.DesiredLinearVelocity = 1.5;
ppController.MaxAngularVelocity = 0.6;

% Controlador VFH para evasión de obstáculos
vfhControl = controllerVFH;
vfhControl.DistanceLimits = [0.05 3];
vfhControl.NumAngularSectors = 36;
vfhControl.HistogramThresholds = [5 10];
vfhControl.RobotRadius = distEjes;
vfhControl.SafetyDistance = distEjes;
vfhControl.MinTurningRadius = 0.1;

%% Bucle de simulación
simRate = rateControl(1/Ts);
for k = 2:numel(tiempo)
    
    % Lecturas del sensor
    estadoActual = trayectoria(:,k-1);
    mediciones = sensorLidar(estadoActual);
    
    % Cálculo de referencia y evasión
    [v,w,obj] = ppController(estadoActual);
    direccionObjetivo = atan2(obj(2)-estadoActual(2), obj(1)-estadoActual(1)) - estadoActual(3);
    direccionEvitada = vfhControl(mediciones, sensorLidar.scanAngles, direccionObjetivo);
    
    if ~isnan(direccionEvitada) && abs(direccionEvitada - direccionObjetivo) > 0.1
        w = 0.5 * direccionEvitada;
    end

    % Cálculo de velocidades y actualización
    velBody = [v; 0; w];
    velWorld = bodyToWorld(velBody, estadoActual);
    trayectoria(:,k) = estadoActual + velWorld * Ts;

    % Visualización
    visor(trayectoria(:,k), puntosRuta, mediciones);
    waitfor(simRate);
end