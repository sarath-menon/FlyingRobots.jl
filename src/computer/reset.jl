



function full_reset!(computer::OnboardComputer)
    reset_clock(computer.main_clock)
    reset_controllers(computer)
    reset_estimators(computer)
end

# to be defined by 
reset_controllers!(computer::OnboardComputer) = error("reset_controllers function has to be defined for each onboard computer")

reset_estimators!(computer::OnboardComputer) = error("reset_estimators function has to be defined for each onboard computer")
