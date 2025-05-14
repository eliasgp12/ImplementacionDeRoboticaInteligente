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
tVec = 0:sampleTime:33;         % Time array

initPose = [2.48;0.68;pi];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [  2.48,  0.68;   % A
               2.05,  0.70;   % C
               1.23,  1.54;   % D
               0.49,  3.06;   % E
               0.47,  2.34;   % F
               0.47,  2.34;   % G (≈F)
               0.06,  2.66;   % H
               0.06,  2.66;   % I (≈H)
              -0.31,  2.58;   % J
              -0.75,  2.40;   % K
              -0.70,  1.91;   % L
              -0.36,  2.31;   % M
              -0.36,  2.31;   % N (≈M)
              -0.75,  2.40;   % O (≈K)
              -1.11,  2.29;   % P
              -1.11,  2.29;   % Q (≈P)
              -1.91,  1.88;   % R
              -1.91,  1.88;   % S (≈R)
              -1.51,  0.71;   % T
              -0.28,  0.65;   % U
               0.08,  0.29;   % V
               0.09, -0.98;   % W
               0.48, -1.69;   % Z
               2.50, -1.71;   % A₁
               2.48,  0.68;   % B₁ (≈A)
               2.05,  0.70;   % B₂ (≈C)
               0.10, -0.11;   % C₁
               0.08,  0.29;   % C₂ (≈V)
               1.66,  1.10 ]; % D₁

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 0.7;
controller.MaxAngularVelocity = 4;

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