


function find_var_pos(var::String, df::DataFrame)
    df_index = findfirst(x -> x == var, names(df))

    # subtract 1 account for timestamp column in df
    state_index = df_index - 1

    return state_index
end


function get_ref_indices(df)

    start_elements_vec = ["(controller.r_ref(t), 1)",
        "(controller.ṙ_ref(t), 1)",
        "(controller.q_ref(t), 1)",
        "(controller.ω_ref(t), 1)"]

    for i in start_elements_vec
        id = find_var_pos(i, df)
        @show i, id
    end

end
