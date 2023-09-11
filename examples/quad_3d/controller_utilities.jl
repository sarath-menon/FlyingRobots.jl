
motor_thrust_to_body_thrust(; l, k_τ) = [1 1 1 1
    0 l 0 -l
    -l 0 l 0
    k_τ -k_τ k_τ -k_τ]

body_thrust_to_motor_thrust(; l, k_τ) = inv(motor_thrust_to_body_thrust(; l=l, k_τ=k_τ))


function load_controller_params(path::String, vehicle_params)
    ctrl_yaml = YAML.load_file(path; dicttype=Dict{Symbol,Any})

    return ctrl_yaml
end


function initialize_task_stack!(computer::OnboardComputer, params_dict)

    computer.ram_memory[:task_mem] = Dict{Symbol,Dict}()
    task_mem = computer.ram_memory[:task_mem]

    # iterate over tasks
    for task in flight_controller.tasks

        # empty stack for task
        task_stack = task_mem[Symbol(task.name)] = Dict{Symbol,Any}()

        type_blocks = params_dict[:strategies][Symbol(task.name)][Symbol(typeof(task.strategy))][:init]

        # run task initilization function if it's defined
        if hasmethod(initialize_task!, Tuple{typeof(task.strategy),OnboardComputer})
            initialize_task!(task.strategy, computer)
        end

        if type_blocks === nothing
            continue
        end

        for type_block in type_blocks

            type_symbol = Symbol(type_block[:type])

            task_stack[type_symbol] = Dict{Symbol,Any}()

            # get variable type
            type = getfield(Main, type_symbol)

            # iterate over task initialization variables
            for (var_name, args) in pairs(type_block[:objects])

                # create object of type
                var = type(; args...)

                # add object to task stack
                task_stack[type_symbol][var_name] = var
            end
        end

        # # create stack space for task strategy
        # task_mem[Symbol(task.name)] = task_stack
    end
end

