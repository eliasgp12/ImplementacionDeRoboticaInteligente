
% Definir parámetros del vehículo
radioRueda = 0.1;                     % Radio de rueda [m]
separacionRuedas = 0.5;              % Distancia entre ruedas [m]
vehiculo = DifferentialDrive(radioRueda, separacionRuedas);

% Tiempo de simulación
deltaT = 0.1;                         
tiempoSim = 0:deltaT:33;              % Vector de tiempo

% Estado inicial del robot
estadoIni = [2; 2; 0];               
estado = zeros(3, numel(tiempoSim)); 
estado(:,1) = estadoIni;

% Cargar mapa
close all
load complexMap

% Sensor Lidar
sensor = LidarSensor;
sensor.sensorOffset = [0, 0];
sensor.scanAngles = linspace(-pi, pi, 200);
sensor.maxRange = 0.5;

% Visualizador
visor2D = Visualizer2D;
visor2D.hasWaypoints = true;
visor2D.mapName = 'map';
attachLidarSensor(visor2D, sensor);

%% Planificación y seguimiento de ruta

% Puntos de referencia (waypoints)
puntosRuta = [estadoIni(1:2)'; 
              7 6;
              9 8;
              2 6;
              4 8;
              9 3;
              7 2];

% Controlador Pure Pursuit
controlPP = controllerPurePursuit;
controlPP.Waypoints = puntosRuta;
controlPP.LookaheadDistance = 0.5;
controlPP.DesiredLinearVelocity = 1.0;
controlPP.MaxAngularVelocity = 200;

% Controlador de evasión de obstáculos VFH
controlVFH = controllerVFH;
controlVFH.DistanceLimits = [0.05 3];
controlVFH.NumAngularSectors = 36;
controlVFH.HistogramThresholds = [5 10];
controlVFH.RobotRadius = separacionRuedas;
controlVFH.SafetyDistance = separacionRuedas;
controlVFH.MinTurningRadius = 0.1;

%% Bucle de simulación
temporizador = rateControl(1/deltaT);
for k = 2:numel(tiempoSim)
    
    % Obtener lectura del sensor y estado actual
    estadoActual = estado(:,k-1);
    lecturaLidar = sensor(estadoActual);
    
    % Algoritmos de control
    [velLineal, velAngular, puntoObjetivo] = controlPP(estadoActual);
    direccionDeseada = atan2(puntoObjetivo(2) - estadoActual(2), puntoObjetivo(1) - estadoActual(1)) - estadoActual(3);
    direccionFinal = controlVFH(lecturaLidar, sensor.scanAngles, direccionDeseada);
    
    if ~isnan(direccionFinal) && abs(direccionFinal - direccionDeseada) > 0.1
        velAngular = 0.5 * direccionFinal;
    end

    % Movimiento y actualización de estado
    velRobot = [velLineal; 0; velAngular];
    velMundo = bodyToWorld(velRobot, estadoActual);
    estado(:,k) = estadoActual + velMundo * deltaT;

    % Mostrar en visualizador
    visor2D(estado(:,k), puntosRuta, lecturaLidar);
    waitfor(temporizador);
end