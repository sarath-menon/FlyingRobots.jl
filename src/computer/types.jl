export ComputerClock, OnboardComputer
export ClockSpeed

# Abstract types -----------------------------

abstract type AbstractComputer end

# Concrete types -----------------------------

mutable struct ComputerClock4
    count::Integer
    speed::Integer
    ComputerClock4(; speed) = new(0, speed)
end

ComputerClock = ComputerClock4

struct OnboardComputer6 <: AbstractComputer
    name::String
    main_clock::ComputerClock
    ram_memory::Dict
    rom_memory::NamedTuple
    tasks::Vector{NamedTuple}

    OnboardComputer6(name; main_clock, ram_memory, rom_memory, tasks) =
        new(name, main_clock, ram_memory, rom_memory, tasks)
end

OnboardComputer = OnboardComputer6

# convenience constructor