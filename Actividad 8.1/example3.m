

% Parámetros del robot diferencial
rueda = 0.1;                       % Radio [m]
distanciaEjes = 0.5;              % Distancia entre ruedas [m]
robotMovil = DifferentialDrive(rueda, distanciaEjes);

% Tiempo de simulación
dt = 0.1;                          % Tiempo de muestreo [s]
tiempoTotal = 0:dt:120;           % Vector temporal

% Posición inicial
estadoInicial = [11.5; 2.5; 0];   
trayectoria = zeros(3, numel(tiempoTotal));
trayectoria(:,1) = estadoInicial;

% Cargar mapa
close all
load exampleMap

% Sensor Lidar
sensorLidar = LidarSensor;
sensorLidar.sensorOffset = [0, 0];
sensorLidar.scanAngles = linspace(0, 2*pi, 200);
sensorLidar.maxRange = 0.55;

% Visualizador
visual = Visualizer2D;
visual.hasWaypoints = true;
visual.mapName = 'map';
attachLidarSensor(visual, sensorLidar);

%% Planificación de Ruta

% Definir puntos de paso (waypoints)
ruta = [estadoInicial(1:2)';
        11.5 2.5;
        8 2.5;
        11.5 5;
        8 5;
        11.5 8.5;
        11.5 12;
        8 12; 
        8 8.5;
        4.5 8.5;
        4.5 12;
        1 12;
        1 8.5;
        4.5 5;
        4.5 2.5; 
        1 2.5;        
        1 5];

% Controlador Pure Pursuit
seguidor = controllerPurePursuit;
seguidor.Waypoints = ruta;
seguidor.LookaheadDistance = 0.3;
seguidor.DesiredLinearVelocity = 0.5;
seguidor.MaxAngularVelocity = 10;

% Controlador VFH para evasión de obstáculos
evitador = controllerVFH;
evitador.DistanceLimits = [0.05 3];
evitador.NumAngularSectors = 36;
evitador.HistogramThresholds = [5 10];
evitador.RobotRadius = distanciaEjes;
evitador.SafetyDistance = distanciaEjes;
evitador.MinTurningRadius = 0.1;

%% Bucle de Simulación
controlTiempo = rateControl(1/dt);
for k = 2:numel(tiempoTotal)
    
    % Obtener estado y lectura del sensor
    estado = trayectoria(:,k-1);
    escaneo = sensorLidar(estado);
    
    % Algoritmos de control y evasión
    [vLineal, wAngular, puntoObjetivo] = seguidor(estado);
    direccionObjetivo = atan2(puntoObjetivo(2)-estado(2), puntoObjetivo(1)-estado(1)) - estado(3);
    direccionModificada = evitador(escaneo, sensorLidar.scanAngles, direccionObjetivo);
    
    if ~isnan(direccionModificada) && abs(direccionModificada - direccionObjetivo) > 0.1
        wAngular = 0.5 * direccionModificada;
    end
    
    % Calcular velocidad global y actualizar posición
    velocidadLocal = [vLineal; 0; wAngular];
    velocidadGlobal = bodyToWorld(velocidadLocal, estado);
    trayectoria(:,k) = estado + velocidadGlobal * dt;

    % Mostrar en pantalla
    visual(trayectoria(:,k), ruta, escaneo);
    waitfor(controlTiempo);
end