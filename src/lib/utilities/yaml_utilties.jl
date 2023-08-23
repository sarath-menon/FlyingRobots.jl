export recursive_dict_to_namedtuple, get_configs_vec

# recursive function to converted nested dict to nested NamedTuple
function recursive_dict_to_namedtuple(val)
    if typeof(val) == Dict{Symbol,Any}
        named_tuple = NamedTuple((Symbol(key), recursive_dict_to_namedtuple(value)) for (key, value) in val)
        return named_tuple

    elseif typeof(val) == Vector{Dict{Symbol,Any}}

        named_tuple_vec = NamedTuple[]
        for i in val
            named_tuple = recursive_dict_to_namedtuple(i)
            push!(named_tuple_vec, named_tuple)
        end
        return named_tuple_vec

    else
        return val
    end
end


function get_axis_titles(configs_vec::Vector{String}, config::String)
    index = findfirst(item -> item == config, configs_vec)

    axis_titles = getindex(plot_params.graph.axis.configs[index], :y_axis_names)

    return axis_titles
end

function get_configs_vec(config_tuple::Vector{NamedTuple})
    configs_vec = String[]

    for (i, config) in enumerate(config_tuple)
        push!(configs_vec, getindex(config, :name))
    end

    return configs_vec
end