let

    df = DataFrame(A=Int[], B=Vector{Float64}[])
    X::Vector{Float64} = [1.0, 2.0]
    @timev begin

        for i in 1:1:100

            push!(df, (1, X))
        end
    end
end

