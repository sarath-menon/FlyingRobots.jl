
export control_cb

control_cb = PeriodicCallback(0.01, initial_affect=true, save_positions=(false, true)) do integrator

    # Extract the parameters
    (; m, l, I_xx, safety_box, K) = integrator.p.quad
    nx, nu, ny, Ts = integrator.p.frmodel
    (; log_matrix) = integrator.p.logger

    # Extract the state 
    X = @view integrator.u[1:nx]

    X_req = generate_trajectory(circle_trajec, quad_obj, integrator.t)

    # compute control input
    X_error = X - X_req
    U = -K * X_error

    # # println("X_req: $(X_req)")
    # #println("X_error: $(X_error)")
    #println("State: $(X)")

    f_1_equilibirum = f_2_equilibirum = -g / 2

    f_1::Float64 = f_1_equilibirum + U[1]
    f_2::Float64 = f_2_equilibirum + U[2]

    # constrain the control input
    f_1 = clamp(f_1, 0.0, 12.5)
    f_2 = clamp(f_2, 0.0, 12.5)

    #Update the control-signal
    integrator.u[nx+1:end] = @SVector [f_1, f_2]

    # # logging
    # write_row_vector!(log_matrix, integrator.u, integrator.t, Ts)

end




