
export actuator_cmd_to_ctrl_cmd_matrix

actuator_cmd_to_ctrl_cmd_matrix(L::Float64) = @SMatrix[1 1; L -L]