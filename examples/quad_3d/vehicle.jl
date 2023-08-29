
function load_vehicle_params_non_computer(path::String)
    vehicle_yaml = YAML.load_file(path; dicttype=Dict{Symbol,Any})

    vehicle_params = recursive_dict_to_namedtuple(vehicle_yaml)

    return vehicle_params
end




function get_initial_conditions(plant, vehicle_params)

    initial_state = vehicle_params.initial_state

    # initial conditions
    r₀ = initial_state.pos # position
    ṙ₀ = initial_state.vel # velocity
    q₀ = QuatRotation(RotZYX(initial_state.orientation.yaw, initial_state.orientation.pitch, initial_state.orientation.roll)) # (Orientation - yaw, pitch, roll)
    ω₀ = initial_state.angular_vel # angular velocity

    X₀ = [collect(plant.rb.r .=> r₀)
        collect(plant.rb.ṙ .=> ṙ₀)
        collect(plant.rb.q .=> [q₀.q.s, q₀.q.v1, q₀.q.v2, q₀.q.v3])
        collect(plant.rb.ω .=> ω₀)]

    return X₀
end

function get_parameters(plant, vehicle_params)
    parameters = [
        plant.rb.m => vehicle_params.mass,
        plant.l => vehicle_params.arm_length,
        plant.k_τ => vehicle_params.actuators.constants.k_τ,
        plant.rb.I_xx => vehicle_params.I_xx,
        plant.rb.I_yy => vehicle_params.I_yy,
        plant.rb.I_zz => vehicle_params.I_zz,
        plant.motor_1.first_order_system.T => vehicle_params.actuators.constants.τ,
        plant.motor_2.first_order_system.T => vehicle_params.actuators.constants.τ,
        plant.motor_3.first_order_system.T => vehicle_params.actuators.constants.τ,
        plant.motor_4.first_order_system.T => vehicle_params.actuators.constants.τ]

    return parameters
end


