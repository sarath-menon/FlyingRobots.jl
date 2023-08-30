module ComputerTest

using FlyingRobots
using FlyingRobots.Computer: ClockSpeed, ComputerClock
using FlyingRobots.Computer: increment_clock!, reset_clock!
using FlyingRobots.Computer: get_elapsed_time

function create_computer(name)

    # clock
    main_clock = ComputerClock(; speed=500)

    # ROM memory
    params = (; mass=2.1)
    rom_memory = (; params=params)

    # RAM memory
    ram_memory = Dict{Symbol,Any}()

    return OnboardComputer(name; main_clock=main_clock, ram_memory=ram_memory,
        rom_memory=rom_memory, tasks=vehicle_params.computer.tasks)
end

# test if creating computer works 
test_computer = create_computer("test_computer")

# print clock details 
show(test_computer.main_clock)

# show time elapsed since bootup
get_elapsed_time(test_computer)

# clock increment
increment_clock!(test_computer.main_clock)

# clock reset 
reset_clock!(test_computer.main_clock)


end