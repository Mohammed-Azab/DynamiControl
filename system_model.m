% requirements
Ts = 2.5;    % settling time
Mp = 20/100; % overshoot
err = 1/100; % steady state error

% Mp = e^(-zeta*pi/(sqrt(1-zeta^2)))

zeta= 0.4559; 
Wn = 4.6/zeta; 
s = tf('s'); % s-doamin
G = (1 / (s^2 + 2*Wn*zeta*s + Wn^2)) * (0.21 / (s^2));














figure;
plot(u_P, '-o', 'DisplayName', 'P Controller'); hold on;
plot(u_PI, '-s', 'DisplayName', 'PI Controller');
plot(u_PD, '-d', 'DisplayName', 'PD Controller');
plot(u_PID, '-^', 'DisplayName', 'PID Controller');
xlabel('Time Step');
ylabel('Control Output (u)');
title('Comparison of P, PI, PD, and PID Controllers');
legend;
grid on;