% Define the plant transfer function (Qp)
numerator = [(0.0043 * 0.177)];  % Example numerator coefficients
denominator = [(0.0360), (0.02), (1.1 * 9.81 * 0.014)];  % Example denominator coefficients
sys = tf(numerator, denominator);  % Plant transfer function

% Define the PID controller parameters
P = 12;  % Proportional gain
I = 0.035;  % Integral gain
D = 1.2;  % Derivative gain
N = 150;  % Filter coefficient
b = 0.02;  % Setpoint weight for proportional term
c = 0.1;  % Setpoint weight for derivative term

% Proportional + Integral term with b weight
PID_PI = tf([b * P, I], [1, 0]);

% Derivative term with c weight
PID_D = tf([D * c * N, 0], [1, N]);

% Full PID transfer function
PID = PID_PI + PID_D;

% Combine PID controller and plant
open_loop_sys = PID * sys;  % Open-loop transfer function

% Generate Bode plot with gain and phase margins
figure;
margin(open_loop_sys);  % Generate Bode plot with margin annotations
grid on;

% Customize the plot
title('Bode Plot with Phase Margin for 2-DOF PID Controller');
