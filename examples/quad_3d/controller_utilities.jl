
motor_thrust_to_body_thrust(; l, k_τ) = [1 1 1 1
    0 l 0 -l
    -l 0 l 0
    k_τ -k_τ k_τ -k_τ]

body_thrust_to_motor_thrust(l, k_τ) = inv(motor_thrust_to_body_thrust(; l=l, k_τ=k_τ))

function load_controller_params(path::String, vehicle_params)
    ctrl_yaml = YAML.load_file(path; dicttype=Dict{Symbol,Any})

    # set controller params 
    allocation_matrix = body_thrust_to_motor_thrust(vehicle_params.arm_length, vehicle_params.actuators.constants.k_τ)
    ctrl_yaml[:allocation_matrix] = allocation_matrix

    return ctrl_yaml
end

function load_pid_gains(path::String)

    # load pid gains
    ctrl_yaml = load_pid_gains(path)

    # set allocation matrix
    allocation_matrix = body_thrust_to_motor_thrust(vehicle_params.arm_length, vehicle_params.actuators.constants.k_τ)

    ctrl_yaml[:allocation_matrix] = allocation_matrix

    return ctrl_yaml
end