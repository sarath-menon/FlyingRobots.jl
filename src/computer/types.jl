export ComputerClock, OnboardComputer

mutable struct ComputerClock1
    count::Int64

    ComputerClock1() = new(0)
end

ComputerClock = ComputerClock1

struct OnboardComputer4
    name::String
    main_clock::ComputerClock
    ram_memory::Dict
    rom_memory::NamedTuple
    tasks::Vector{NamedTuple}

    OnboardComputer4(name; main_clock, ram_memory, rom_memory, tasks) =
        new(name, main_clock, ram_memory, rom_memory, tasks)
end

OnboardComputer = OnboardComputer4

# convenience constructor