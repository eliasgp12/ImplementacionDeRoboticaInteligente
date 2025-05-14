%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.2;               % Sample time [s]
tVec = 0:sampleTime:45;         % Time array

initPose = [0.06;-1.36;pi/4];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [  0.06, -1.36;
               0.73, -0.71;
               1.41, -0.69;
               0.75, -1.38;
              -0.63, -1.37;
              -1.25, -0.71;
              -0.63, -0.69;
               0.06, -1.36;
               0.06, -0.08;
              -0.63, -0.69;
              -0.61, -0.01;
              -1.25, -0.02;
              -0.63,  0.65;
              -1.27,  1.32;
              -0.62,  1.34;
              -0.62,  2.01;
               0.07,  1.34;
               0.73,  2.01;
               0.74,  1.33;
               1.40,  1.34;
               0.76,  0.65;
               1.42, -0.01;
               0.75, -0.02;
               0.73, -0.68;
               0.07, -0.06;
               0.06,  0.31;
              -0.27,  0.32;
              -0.26,  0.99;
               0.40,  0.99;
               0.40,  0.31;
               0.06,  0.31];

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 3;

%% Simulation loop
close all
r = rateControl(1/sampleTime);

for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
 
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints);
    waitfor(r);
    
end