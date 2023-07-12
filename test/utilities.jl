export check_if_struct_equals_vector

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

