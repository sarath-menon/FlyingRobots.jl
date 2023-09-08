export pid_controller!, reset!, pd_tuning, update!
export PID

mutable struct PID1
    kp::Float64 # proportional gain
    ki::Float64 # integral gain
    kd::Float64 # derivative gain

    k_aw::Float64 # integral antiwindup gain

    error_integral::Float64 # integral accumulated over time
    prev_error::Float64 # error from prvious time step

    function PID1(; kp=kp, ki=ki, kd=kd, k_aw=k_aw)
        new(kp, ki, kd, k_aw, 0.0, 0.0)
    end

    function PID1(dict::Dict)
        new(dict[:k_p], dict[:k_i], dict[:k_d], dict[:k_aw], 0.0, 0.0)
    end

end

PID = PID1

function pid_controller!(pid::PID; e, dt, umin, umax)
    # parameters
    kp, ki, kd = pid.kp, pid.ki, pid.kd
    h, k_aw = dt, pid.k_aw

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

function update!(pid::PID, dict::Dict)
    # parameters
    pid.kp = dict[:k_p]
    pid.ki = dict[:k_i]
    pid.kd = dict[:k_d]
    pid.k_aw = dict[:k_aw]
end

function reset!(pid::PID)
    pid.error_integral = 0
    pid.prev_error = 0
end

function pd_tuning(; response, ζ=0, τ=0)

    if response == :second_order

        kp = 1 / τ^2
        kd = 2 * ζ / τ

    elseif response == :first_order
        kp = 1 / τ^2
        kd = 0

    else
        println("reponse type is invalid: only :first_order and :second order are supported")

    end
    @show kp, kd
end
