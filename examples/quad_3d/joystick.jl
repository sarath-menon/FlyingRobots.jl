module Joystick

using GLFW
# using GLMakie
# GLMakie.activate!(inline=false)

export JSDevice, JoystickState
export connect_joystick, get_joystick_state

struct JSDevice1
    name::String
    device::GLFW.Joystick
    axis_count::Int32
    button_count::Int32

    JSDevice1(name; device, axis_count, button_count) = new(name, device, axis_count, button_count)
end

JSDevice = JSDevice1

struct JoystickState1
    horizontal_axis::Float64
    vertical_axis::Float64
    stick_rotate_axis::Float64
    knob_rotate_axis::Float64

    buttons::Vector{UInt8}

    JoystickState1(; horizontal_ax, vertical_ax, stick_rotate_ax, knob_rotate_ax, buttons) = new(horizontal_ax, vertical_ax, stick_rotate_ax, knob_rotate_ax, buttons)
end

JoystickState = JoystickState1


function connect_joystick(device=GLFW.JOYSTICK_1)
    if GLFW.JoystickPresent(device) == false
        println("Joytick not connected !")
    else
        axis_count = length(GLFW.GetJoystickAxes(device))
        button_count = length(GLFW.GetJoystickButtons(device))

        joystick = JSDevice("logitech_flight_stick"; device=device, axis_count=axis_count, button_count=button_count)

        return joystick

    end
end

function get_joystick_state(js)
    # get axis values

    axis_vals = GLFW.GetJoystickAxes(js.device)
    buttons_state_uint8 = GLFW.GetJoystickButtons(js.device)

    vertical_ax_val = -axis_vals[2]
    horizontal_ax_val = axis_vals[1]

    stick_rotate_ax_val = -axis_vals[3]
    knob_rotate_ax_val = -axis_vals[4]

    js_state = JoystickState(horizontal_ax=horizontal_ax_val, vertical_ax=vertical_ax_val, stick_rotate_ax=stick_rotate_ax_val,
        knob_rotate_ax=knob_rotate_ax_val, buttons=buttons_state_uint8)

    return js_state
end

function Base.show(js_state::JoystickState)

    println("Horizontal axis: ", js_state.horizontal_axis)
    println("Vertical axis: ", js_state.vertical_axis)
    println("Stick rotate axis: ", js_state.stick_rotate_axis)
    println("Knob rotate axis: ", js_state.knob_rotate_axis)

end

end