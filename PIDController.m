Kp = 0.5;
Ki = 0.5;
kd = 0.5;
dt = 0.01; % Time step


function u = PID(e,p,i,d,kp,ki,kd)
    
    persistent integral_error prev_error;
    
    if isempty(integral_error)
        integral_error = 0;
    end
    if isempty(prev_error)
        prev_error = 0;
    end
    P = 0;
    u = P + I + D;

    prev_error = e;


end


function u = P(e,p,i,d,kp,ki,kd)
    u = PID(e,p,i,d,kp,0,0);

end

function u = PI(e,p,i,d,kp,ki,kd)
     u = PID(e,p,i,d,kp,ki,0);

end

function u = PD(e,p,i,d,kp,ki,kd)
     u = PID(e,p,i,d,kp,0,kd);

end



