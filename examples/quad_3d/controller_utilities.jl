
motor_thrust_to_body_thrust(; l, k_τ) = [1 1 1 1
    0 l 0 -l
    -l 0 l 0
    k_τ -k_τ k_τ -k_τ]

body_thrust_to_motor_thrust(l, k_τ) = inv(motor_thrust_to_body_thrust(; l=l, k_τ=k_τ))

function load_controller_params(path::String, vehicle_params)
    ctrl_yaml = YAML.load_file(path; dicttype=Dict{Symbol,Any})

    # set controller params 
    allocation_matrix = body_thrust_to_motor_thrust(vehicle_params.arm_length, vehicle_params.actuators.constants.k_τ)
    ctrl_yaml[:allocation_matrix] = allocation_matrix

    return ctrl_yaml
end

function load_pid_gains(path::String)

    # load pid gains
    ctrl_yaml = load_pid_gains(path)

    # set allocation matrix
    allocation_matrix = body_thrust_to_motor_thrust(vehicle_params.arm_length, vehicle_params.actuators.constants.k_τ)

    ctrl_yaml[:allocation_matrix] = allocation_matrix

    return ctrl_yaml
end


function initialize_task_stack!(ram_memory::Dict, params_dict)

    ram_memory[:task_mem] = Dict{Symbol,Dict}()
    task_mem = ram_memory[:task_mem]

    # iterate over tasks
    for task in flight_controller.tasks
        # empty stack for task
        task_stack = Dict{Symbol,Any}()

        # @show Symbol(task.name), Symbol(typeof(P_PosController()))

        type_blocks = params_dict[:strategies][Symbol(task.name)][Symbol(typeof(task.strategy))][:init]

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

        # create stack space for task strategy
        task_mem[Symbol(task.name)] = task_stack
    end
end


function initialize_task_stack!(computer::OnboardComputer, params_dict)
    initialize_task_stack!(computer.ram_memory, params_dict)
end

# function load_strategy(computer, params_dict; task_name)
#     tasks = params_dict[:vehicle][:computer][:tasks]

#     computer.ram_memory[:task_mem] = Dict{Symbol,Dict}()
#     task_mem = computer.ram_memory[:task_mem]

#     # iterate over tasks
#     for task in tasks
#         if task.name == task_name
#             # @show task.name

#             # get task function
#             task_func = getfield(Main, Symbol(task[:name]))

#             @show task_func

#             # empty stack for task
#             task_stack = Dict{Symbol,Any}()

#             # iterate over task strategies
#             for strategy in task.strategies.lib
#                 if strategy.name == task.strategies.selected
#                     println("Selcted strategy: ", task.strategies.selected)

#                     # iterate over task initialization variables
#                     for var in strategy.init

#                         # get variable type
#                         type = getfield(Main, Symbol(var.type))

#                         for (var_name, args) in pairs(var.objects)

#                             # create object of type
#                             var = type(; args...)

#                             # add object to task stack
#                             task_stack[var_name] = var
#                         end
#                     end
#                 end
#             end

#             # create stack space for task strategy
#             task_mem[Symbol(task.name)] = task_stack
#         end
#     end
# end