

% Definición del vehículo diferencial
radioR = 0.1;                   % Radio de la rueda [m]
longitudEje = 0.5;              % Distancia entre ruedas [m]
robotDif = DifferentialDrive(radioR, longitudEje);

% Tiempo de muestreo y vector de simulación
ts = 0.1;                       
tiempo = 0:ts:52;               

% Condiciones iniciales
estadoInicial = [4; 6; pi/6];  
estados = zeros(3, numel(tiempo));
estados(:,1) = estadoInicial;

% Cargar el mapa
close all
load complexMap

% Configurar sensor lidar
lidarSim = LidarSensor;
lidarSim.sensorOffset = [0, 0];
lidarSim.scanAngles = linspace(0, 2*pi, 500);
lidarSim.maxRange = 0.845;

% Configurar visualizador
visor = Visualizer2D;
visor.hasWaypoints = true;
visor.mapName = 'map';
attachLidarSensor(visor, lidarSim);

%% Planificación de Ruta

% Definición de los waypoints para la trayectoria
trayecto = [estadoInicial(1:2)';
            4 6;
            9 6;
            9 2;
            9 6;
            4 2];

% Controlador de seguimiento Pure Pursuit
controlSeguidor = controllerPurePursuit;
controlSeguidor.Waypoints = trayecto;
controlSeguidor.LookaheadDistance = 0.4;
controlSeguidor.DesiredLinearVelocity = 0.5;
controlSeguidor.MaxAngularVelocity = 2;

% Controlador VFH para evitar obstáculos
controlVFH = controllerVFH;
controlVFH.DistanceLimits = [0.05 3];
controlVFH.NumAngularSectors = 36;
controlVFH.HistogramThresholds = [5 10];
controlVFH.RobotRadius = longitudEje;
controlVFH.SafetyDistance = longitudEje;
controlVFH.MinTurningRadius = 0.1;

%% Bucle de Simulación
temporizador = rateControl(1/ts);
for paso = 2:numel(tiempo)

    % Leer estado actual y escaneo Lidar
    estadoActual = estados(:, paso - 1);
    escaneo = lidarSim(estadoActual);
    
    % Calcular referencia y dirección de evasión
    [v, w, puntoObjetivo] = controlSeguidor(estadoActual);
    direccionDeseada = atan2(puntoObjetivo(2) - estadoActual(2), puntoObjetivo(1) - estadoActual(1)) - estadoActual(3);
    direccionLimpia = controlVFH(escaneo, lidarSim.scanAngles, direccionDeseada);
    
    if ~isnan(direccionLimpia) && abs(direccionLimpia - direccionDeseada) > 0.1
        w = 0.5 * direccionLimpia;
    end

    % Cálculo de velocidades y avance
    velLocal = [v; 0; w];
    velMundo = bodyToWorld(velLocal, estadoActual);
    estados(:, paso) = estadoActual + velMundo * ts;

    % Actualización de la visualización
    visor(estados(:, paso), trayecto, escaneo);
    waitfor(temporizador);
end