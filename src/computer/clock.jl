
export increment_clock, reset_clock

"""
basins_fractions(basins::AbstractArray) → fs::Dict

Calculate interest from an `amount` and interest rate of `rate`.
"""

function increment_clock!(clock::ComputerClock)
    clock.count += 1
end

# return time elapsed since bootup in seconds
function get_elapsed_time(computer::OnboardComputer)
    count = computer.main_clock.count
    clock_speed = computer.main_clock.speed

    return count / clock_speed
end

function Base.show(clock::ComputerClock)
    println("Clock speed is $(clock.speed) Hz")
end


function reset_clock!(clock::ComputerClock)
    clock.count = 0
end

