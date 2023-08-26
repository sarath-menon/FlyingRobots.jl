mutable struct ComputerClock1
    count::Int64

    ComputerClock1() = new(0)
end

ComputerClock = ComputerClock1