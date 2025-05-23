
% Definición del robot diferencial
radio = 0.1;                        % Radio de las ruedas [m]
distanciaEjes = 0.5;                % Distancia entre ruedas [m]
robot = DifferentialDrive(radio, distanciaEjes);

% Tiempo de muestreo y vector temporal
Ts = 0.1;                           
tiempo = 0:Ts:55.5;                

% Condiciones iniciales
estadoIni = [2; 4; -pi/2];         
trayectoria = zeros(3, numel(tiempo));
trayectoria(:,1) = estadoIni;

% Cargar el mapa
close all
load complexMap

% Configurar sensor Lidar
sensorLidar = LidarSensor;
sensorLidar.sensorOffset = [0, 0];
sensorLidar.scanAngles = linspace(0, 2*pi, 200);
sensorLidar.maxRange = 0.92;

% Configurar visualizador
visor = Visualizer2D;
visor.hasWaypoints = true;
visor.mapName = 'map';
attachLidarSensor(visor, sensorLidar);

%% Planificación de trayectoria

% Puntos de referencia a seguir
rutas = [estadoIni(1:2)';
         8 4; 
         8 9; 
         2 9; 
         2 14; 
         8 14; 
         2 19; 
         8 19; 
         8 14; 
         16 14;
         16 19; 
         24 19; 
         24 14; 
         24 9;
         16 9; 
         16 4; 
         24 4];

% Controlador de trayectoria Pure Pursuit
seguidor = controllerPurePursuit;
seguidor.Waypoints = rutas;
seguidor.LookaheadDistance = 0.3;
seguidor.DesiredLinearVelocity = 2.0;
seguidor.MaxAngularVelocity = 30;

% Controlador VFH (Vector Field Histogram)
evitador = controllerVFH;
evitador.DistanceLimits = [0.05 3];
evitador.NumAngularSectors = 36;
evitador.HistogramThresholds = [5 10];
evitador.RobotRadius = distanciaEjes;
evitador.SafetyDistance = distanciaEjes;
evitador.MinTurningRadius = 0.1;

%% Bucle de Simulación
temporizador = rateControl(1/Ts);
for i = 2:numel(tiempo)
    
    % Lectura del estado actual y datos del Lidar
    estado = trayectoria(:, i-1);
    lectura = sensorLidar(estado);
    
    % Seguimiento de trayectoria + evasión de obstáculos
    [v, w, objetivo] = seguidor(estado);
    direccion = atan2(objetivo(2) - estado(2), objetivo(1) - estado(1)) - estado(3);
    direccionEvitada = evitador(lectura, sensorLidar.scanAngles, direccion);
    
    disp(direccionEvitada)
    if ~isnan(direccionEvitada) && abs(direccionEvitada - direccion) > 0.1
        w = 0.5 * direccionEvitada;
    end

    % Actualizar posición del robot
    velLocal = [v; 0; w];
    velGlobal = bodyToWorld(velLocal, estado);
    trayectoria(:, i) = estado + velGlobal * Ts;

    % Visualización
    visor(trayectoria(:, i), rutas, lectura);
    waitfor(temporizador);
end