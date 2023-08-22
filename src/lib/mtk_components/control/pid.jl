
mutable struct PID2
    kp::Float64 # proportional gain
    ki::Float64 # integral gain
    kd::Float64 # derivative gain

    k_aw::Float64 # integral antiwindup gain

    Ts::Float64 # sampling time

    error_integral::Float64 # integral accumulated over time
    prev_error::Float64 # error from prvious time step

    function PID2(; kp=kp, ki=ki, kd=kd, k_aw=k_aw, Ts=Ts)
        new(kp, ki, kd, k_aw, Ts, 0.0, 0.0)
    end

end

PID = PID2

function pid_controller(pid::PID; e, umin, umax)
    # parameters
    kp, ki, kd = pid.kp, pid.ki, pid.kd
    h, k_aw = pid.Ts, pid.k_aw

    # compute error terms
    e_dot = (e - pid.prev_error) / h

    e_integral = pid.error_integral

    # compute the control action 
    u = kp * e + kd * e_dot + ki * e_integral

    # limit output to actuator limits 
    u_sat = clamp(u, umin, umax)

    # update integral state 
    pid.error_integral += e * h + k_aw * (u_sat - u)

    # update derivative state 
    pid.prev_error = e

    return u_sat
end