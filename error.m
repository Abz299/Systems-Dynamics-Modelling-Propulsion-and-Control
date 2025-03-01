% Define the plant transfer function (Qp)
numerator = [kt * l_h];  % Example numerator coefficients
denominator = [I_xx, 0.02, m * g * l_theta];  % Example denominator coefficients
sys = tf(numerator, denominator);  % Plant transfer function

% Define the PID controller parameters
P = 0.5;  % Proportional gain
I = 0.004;  % Integral gain
D = 0.1;  % Derivative gain
N = 100;  % Filter coefficient
b = 3;  % Setpoint weight for proportional term
c = 0.5;  % Setpoint weight for derivative term

% Proportional + Integral term with b weight
PID_PI = tf([b * P, I], [1, 0]);

% Derivative term with c weight
PID_D = tf([D * c * N, 0], [1, N]);

% Full PID transfer function
PID = PID_PI + PID_D;

% Combine PID controller and plant
open_loop_sys = PID * sys;  % Open-loop transfer function

% Step response of the combined system
figure;
step(open_loop_sys);  % Step response
grid on;

% Customize the plot
title('Step Response of the Combined System');
xlabel('Time (s)');
ylabel('Amplitude');

% Evaluate step response characteristics
step_info = stepinfo(open_loop_sys);
disp(step_info);  % Display step response characteristics

% Calculate and check if the response converges within 10% error
final_value = step_info.SettlingMin + step_info.SettlingMax;  % Approximation of steady state
error_threshold = abs(1 - final_value);  % Calculate the error
if error_threshold <= 0.1
    disp('Step response converges within 10% error to the desired value.');
else
    disp('Step response does not converge within 10% error to the desired value.');
end
