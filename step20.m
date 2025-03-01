%% System Parameters
M = 3.57; % Total mass (kg)
l_boom = 0.66; % Lever arm for pitch (m)
l_theta = 0.014; % Length for pitch pendulum (m)
I_yy = 0.93; % Moment of inertia about the pitch axis (Nm)
K_T = 4.25e-3; % Thrust coefficient (N/s)
g = 9.81; % Gravitational acceleration (m/s^2)
Mgl_theta = M * g * l_theta; % Gravitational torque about the pitch axis

%% State-Space Representation
% Define state-space matrices
A = [0, 1; -Mgl_theta/I_yy, -0.1]; % Add damping to stabilize the system
B = [0; K_T * l_boom / I_yy];      % Input matrix
C = [1, 0];                        % Output matrix
D = 0;                             % Direct transmission matrix

% Create state-space model
sys_travel = ss(A, B, C, D);

%% Step Input Simulation
step_amplitude = 20 * pi / 180; % Convert 20° to radians

% Simulate step response
figure;
step(step_amplitude * sys_travel, 10); % Simulate for 10 seconds
title('Step Response for 20° Travel Angle Command');
xlabel('Time (s)');
ylabel('Travel Angle (rad)');
grid on;

% Add target line
hold on;
yline(step_amplitude, 'r--', '20° Target (0.349 rad)', 'LineWidth', 1.5);
legend('System Response', 'Target Angle (20°)');
hold off;
