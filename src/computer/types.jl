export ComputerClock, OnboardComputer
export ClockSpeed

# Abstract types -----------------------------

abstract type AbstractComputer end

# Concrete types -----------------------------

mutable struct ComputerClock6
    count::Int
    speed::Int
    ComputerClock6(; speed) = new(0, speed)
end

ComputerClock = ComputerClock6

struct OnboardComputer7 <: AbstractComputer
    name::String
    main_clock::ComputerClock
    ram_memory::Dict
    rom_memory::NamedTuple
    tasks::Vector{NamedTuple}

    OnboardComputer7(name; main_clock, ram_memory, rom_memory, tasks) =
        new(name, main_clock, ram_memory, rom_memory, tasks)
end

OnboardComputer = OnboardComputer7

# convenience constructor