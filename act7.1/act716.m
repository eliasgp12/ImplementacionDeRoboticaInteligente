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
tVec = 0:sampleTime:26;         % Time array

initPose = [1.52;1.38;pi];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [  1.52,  1.38;   % A
              -0.57,  1.38;   % C
               0.11,  2.10;   % D
               0.83,  2.10;   % E
               1.52,  1.38;   % F (≈A)
               0.79,  0.37;   % G
               0.50,  0.66;   % H
              -0.20,  0.66;   % I
              -0.90, -0.02;   % J
              -0.91, -0.76;   % K
              -0.21, -1.45;   % L
               0.50, -1.50;   % M
               1.19, -0.74;   % N
               1.19, -0.02;   % O
               0.79,  0.37;   % P (≈G)
               0.50, -0.01;   % Q
               0.85, -0.04 ]; % R

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