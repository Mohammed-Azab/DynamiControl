% Requirements
Ts = 2.5;    % Settling time
Mp = 20/100; % Overshoot
err = 1/100; % Steady-state error

% Mp = e^(-zeta*pi/(sqrt(1-zeta^2)))

zeta = 0.4559; 
Wn = 4.6/zeta; 
s = tf('s'); % s-domain
G = (1 / (s^2 + 2*Wn*zeta*s + Wn^2)) * (0.21 / (s^2)); % Open loop TF

Kp = 0.6;
Ki = 0.87;
Kd = 0.68;
dt = 0.01; % Time step
t = 0:dt:10000; % Time vector for simulation

% Initialize error and control output arrays
e_p = ones(size(t)); 
p_p = zeros(size(t)); 
i_p = zeros(size(t));
d_p = zeros(size(t)); 

u_P = zeros(size(t));
u_PI = zeros(size(t));
u_PD = zeros(size(t));
u_PID = zeros(size(t));

% Simulate the controllers over time
for i = 2:length(t)
     % Sinusoidal input (e.g., 1 * sin(2*pi*frequency*t))
    frequency = 0.05; % Frequency of the sinusoidal signal (adjust as needed)
    e_p(i) = sin(2 * pi * frequency * t(i)); % Sinusoidal error signal

    i_p(i) = i_p(i-1) + e_p(i)*dt; % Integral of error
    d_p(i) = (e_p(i) - e_p(i-1)) / dt; % Derivative of error
    
    % Call the controller functions (P, PI, PD, PID)
    u_P(i) = PID(e_p(i), p_p(i), i_p(i), d_p(i), Kp, 0, 0);
    u_PI(i) = PID(e_p(i), p_p(i), i_p(i), d_p(i), Kp, Ki, 0);
    u_PD(i) = PID(e_p(i), p_p(i), i_p(i), d_p(i), Kp, 0, Kd);
    u_PID(i) = PID(e_p(i), p_p(i), i_p(i), d_p(i), Kp, Ki, Kd);
end

% Plot the control outputs
figure;
plot(t, u_P, '-o', 'DisplayName', 'P Controller'); hold on;
plot(t, u_PI, '-s', 'DisplayName', 'PI Controller');
plot(t, u_PD, '-d', 'DisplayName', 'PD Controller');
plot(t, u_PID, '-^', 'DisplayName', 'PID Controller');
xlabel('Time Step');
ylabel('Control Output (u)');
title('Comparison of P, PI, PD, and PID Controllers');
legend;
grid on;

% P Controller
Kp_tf = Kp; % P controller gain
G_p = feedback(G*Kp_tf, 1); % Closed-loop transfer function for P controller

% PI Controller
Kpi_tf = Kp + Ki/s; % PI controller transfer function
G_pi = feedback(G*Kpi_tf, 1); % Closed-loop transfer function for PI controller

% PD Controller
Kpd_tf = Kp + Kd*s; % PD controller transfer function
G_pd = feedback(G*Kpd_tf, 1); % Closed-loop transfer function for PD controller

% PID Controller
Kpid_tf = Kp + Ki/s + Kd*s; % PID controller transfer function
G_pid = feedback(G*Kpid_tf, 1); % Closed-loop transfer function for PID controller

% Simulate the closed-loop step response for each controller
figure;
step(G_p, t);
hold on;
step(G_pi, t);
step(G_pd, t);
step(G_pid, t);
xlabel('Time');
ylabel('System Output');
title('Closed-Loop Step Response with Different Controllers');
legend('P Controller', 'PI Controller', 'PD Controller', 'PID Controller');
grid on;

% PID controller function definition
function u = PID(e, p, i, d, kp, ki, kd)
    persistent integral_error prev_error;
    
    if isempty(integral_error)
        integral_error = 0;
    end
    if isempty(prev_error)
        prev_error = 0;
    end

    P = kp * p;
    I = ki * i;
    D = kd * d;

    u = P + I + D;

    prev_error = e;
end
