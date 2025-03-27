% PID controller function
function u = PID(e, p, i, d, kp, ki, kd)
    persistent integral_error prev_error;
    
    if isempty(integral_error)
        integral_error = 0;
    end
    if isempty(prev_error)
        prev_error = 0;
    end

    % Proportional, Integral, and Derivative terms
    P = kp * p;
    I = ki * i;
    D = kd * d;

    % Total control output
    u = P + I + D;
    
    % Update the previous error
    prev_error = e;
end

% P controller function
function u = P(e, p, i, d, kp, ki, kd)
    u = PID(e, p, i, d, kp, 0, 0);
end

% PI controller function
function u = PI(e, p, i, d, kp, ki, kd)
    u = PID(e, p, i, d, kp, ki, 0); 
end

% PD controller function
function u = PD(e, p, i, d, kp, ki, kd)
    u = PID(e, p, i, d, kp, 0, kd);
end
