%% Seguimiento de waypoints con VFH ajustado - exampleMap

% Robot
R = 0.1;
L = 0.5;
robot = DifferentialDrive(R, L);

% Tiempo
sampleTime = 0.1;
tVec = 0:sampleTime:60;% Robot
R = 0.1;
L = 0.5;
robot = DifferentialDrive(R, L);

% Tiempo
sampleTime = 0.1;
tVec = 0:sampleTime:105;

% Estado inicial
initPose = [1; 2; 0];
pose = zeros(3, numel(tVec));
pose(:,1) = initPose;

% Cargar mapa
close all
load exampleMap

% Sensor Lidar
lidar = LidarSensor;
lidar.sensorOffset = [0, 0];
lidar.scanAngles = linspace(-pi, pi, 400);  % Resolución alta para mejor detección de obstáculos
lidar.maxRange = 2.0;                       % Alcance extendido para anticiparse a paredes

% Visualizador
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz, lidar);

% Waypoints
waypoints = [1 2;
             2 10;
             11 8;
             8 2;
             11 8;
             2 10;
             1 2];

% Controlador Pure Pursuit
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.60;         % Distancia de anticipación para suavidad en el trayecto
controller.DesiredLinearVelocity = 0.6;      % Velocidad lenta para evitar colisiones
controller.MaxAngularVelocity = 1.5;         % Giro moderado para estabilidad

% VFH para evasión de obstáculos
vfh = controllerVFH;
vfh.DistanceLimits = [0.1 3];
vfh.NumAngularSectors = 72;                  % Aumenta resolución angular del histograma
vfh.HistogramThresholds = [2 10];
vfh.RobotRadius = L;
vfh.SafetyDistance = 0.3;                    % Distancia mínima a mantener de obstáculos
vfh.MinTurningRadius = 0.8;                  % Controla qué tan cerrado puede girar el robot

% Bucle de simulación
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)
    curPose = pose(:, idx-1);
    ranges = lidar(curPose);

    [vRef, wRef, lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2) - curPose(2), lookAheadPt(1) - curPose(1)) - curPose(3);
    steerDir = vfh(ranges, lidar.scanAngles, targetDir);

    if ~isnan(steerDir) && abs(steerDir - targetDir) > 0.1
        wRef = 0.5 * steerDir;             % Ajuste de giro si hay obstáculo en la dirección original
    end

    velB = [vRef; 0; wRef];
    vel = bodyToWorld(velB, curPose);
    pose(:,idx) = curPose + vel * sampleTime;

    viz(pose(:,idx), waypoints, ranges);
    waitfor(r);
end

% Estado inicial
initPose = [1; 2; 0];
pose = zeros(3, numel(tVec));
pose(:,1) = initPose;

% Cargar mapa
close all
load exampleMap

% Lidar ajustado
lidar = LidarSensor;
lidar.sensorOffset = [0, 0];
lidar.scanAngles = linspace(-pi, pi, 400);
lidar.maxRange = 2.0;  % Mayor alcance para detectar paredes a tiempo

% Visualizador
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz, lidar);

% Waypoints
waypoints = [1 2;
             2 10;
             11 8;
             8 2;
           
             1 2];

% Controlador Pure Pursuit
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.60;
controller.DesiredLinearVelocity = 0.5; % Más lento para evitar choques
controller.MaxAngularVelocity = 1.5;

% VFH mejorado
vfh = controllerVFH;
vfh.DistanceLimits = [0.1 3];          
vfh.NumAngularSectors = 72;            % Mayor resolución angular
vfh.HistogramThresholds = [2 10];      
vfh.RobotRadius = L;
vfh.SafetyDistance = 0.3;              % Mayor distancia de seguridad
vfh.MinTurningRadius = 0.8;            % Más realista para esquivar

% Simulación
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)
    curPose = pose(:, idx-1);
    ranges = lidar(curPose);
    
    [vRef, wRef, lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2) - curPose(2), lookAheadPt(1) - curPose(1)) - curPose(3);
    steerDir = vfh(ranges, lidar.scanAngles, targetDir);
    
    if ~isnan(steerDir) && abs(steerDir - targetDir) > 0.1
        wRef = 0.5 * steerDir;
    end
    
    velB = [vRef; 0; wRef];
    vel = bodyToWorld(velB, curPose);
    pose(:,idx) = curPose + vel * sampleTime;
    
    viz(pose(:,idx), waypoints, ranges);
    waitfor(r);
end