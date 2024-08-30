m = 1;        % Mass (kg)
c = 0.5;      % Damping coefficient (Ns/m)
k = 10;       % Spring constant (N/m)

% State-space representation
A = [0 1; -k/m -c/m];
B = [0; 1/m];
C = [1 0];
D = 0;

% Internal Model for Reference and Disturbance (sinusoidal)
omega = 2*pi;  % Frequency of the sinusoidal signal
A_im = [0 1; -omega^2 0]; % Internal model dynamics for sinusoids
B_im = [0; 1];
C_im = [1 0];

% Augmented System (including internal model)
A_aug = [A zeros(2, 2); B_im*C zeros(2, 2) + A_im];
B_aug = [B; B_im];
C_aug = [C zeros(1, 2)];

% State-feedback control (K is gain for the augmented system)
% Place poles for the augmented system
desired_poles = [-2 -3 -1+1i*omega -1-1i*omega];
K_aug = place(A_aug, B_aug, desired_poles);

% Simulation Parameters
T = 10;        % Simulation time (seconds)
dt = 0.01;     % Time step
t = 0:dt:T;    % Time vector

% Initial Conditions
x_aug = zeros(4, 1);  % Initial state of the augmented system

% Reference and Disturbance
r = sin(omega*t);      % Reference signal
d = 0.1*sin(omega*t);  % Disturbance

% Simulation Loop
y = zeros(1, length(t));  % Output
u = zeros(1, length(t));  % Control input

for i = 1:length(t)
    % Control law (u = -K_aug * x_aug)
    u(i) = -K_aug * x_aug;
    
    % System dynamics (x_dot = A_aug*x_aug + B_aug*u)
    x_dot_aug = A_aug * x_aug + B_aug * u(i) + [0; 0; 0; d(i)];
    
    % Update state
    x_aug = x_aug + x_dot_aug * dt;
    
    % Output
    y(i) = C_aug * x_aug;
end

% Plot Results
figure;
subplot(3,1,1);
plot(t, r, '--', 'LineWidth', 1.5); hold on;
plot(t, y, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Output y(t)');
title('System Output vs Reference Signal');
legend('Reference Signal', 'System Output');

subplot(3,1,2);
plot(t, u, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Input u(t)');
title('Control Input');

subplot(3,1,3);
plot(t, d, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Disturbance d(t)');
title('Disturbance');
