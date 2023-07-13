import Base.==


# High level functions
export convert_Fr_types_to_diffeq_state, struct_to_float64_vector

# Low level functions 
export check_if_struct_equals_vector, checkif_struct_has_only_Float64, get_struct_length, struct_to_float64_vector
export fr_create

#= Convenience function for creating types from FlyingRobots.jl. Creates structs using
an API like that of namedtuples to avoid mixing up fields  =#
function fr_create(type_; kwargs...)
    params_tuple = values(kwargs)
    structfromnt(type_, params_tuple)
end

#= Check if the elements of a struct equal the elements of a vector of ony Float64 elements  =#
function check_if_struct_equals_vector(struct_, vector_::Vector{Float64})

    is_equal = true

    # iterate through struct elements 
    for (i, element) in enumerate(fieldnames(typeof(struct_)))

        val = getfield(struct_, element)

        # check if struct element matches corresponding array valyes
        if val != vector_[i]
            is_equal = false
            break
        end
    end

    return is_equal
end

#= Check if a struct contaons onlu Float64 elements =#
function checkif_struct_has_only_Float64(struct_type)
    for type in struct_type.types
        if type == Float64
            continue
        else
            return false
        end
    end

    return true
end

#= get the number of elements in a struct =#
get_struct_length(struct_type) = length(struct_type.types)

#= get a vector of Float64 elements containing that same values as an input struct 
    of only Float64 elements  =#
function struct_to_float64_vector(struct_)

    #  get type of struct 
    struct_type = typeof(struct_)

    # check if struct contain only float64 elements 
    if checkif_struct_has_only_Float64(struct_type) == false
        println("Cannot convert struct to Float64 vector: Struct contain non-Float64 elements")
    end

    # get number of elements
    len = get_struct_length(struct_type)

    # allocate vector
    vec = Array{Float64}(undef, len)

    # iterate through struct elements 
    for (i, element) in enumerate(fieldnames(struct_type))
        val = getfield(struct_, element)
        vec[i] = val
    end

    return vec
end

function convert_Fr_types_to_diffeq_state(state::FrState, ctrl_cmd::FrCtrlCmd)

    state_vec = struct_to_float64_vector(state)
    control_vec = struct_to_float64_vector(ctrl_cmd)

    diffeq_state = vcat(state_vec, control_vec)
end

#= convert a vector  of Float64} to a type from FlyingRobots.jl =#
fr(vec::Float64, type) = type(vec...)

#= Overlead == function to check whether the values of two structs of the same type are equal=#
function ==(a::T, b::T) where {T<:FrState}
    f = fieldnames(T)
    getfield.(Ref(a), f) == getfield.(Ref(b), f)
end