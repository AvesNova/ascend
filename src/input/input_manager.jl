using GLFW, YAML, Overseer, StaticArrays, DataStructures

include("../rendering/rendering_manager.jl")

const MAX_JOYSTICKS = 15

@enum AxisEnums begin
    PITCH = 1
    YAW
    ROLL
    FLAPS
    SPEEDBRAKE
    THROTTLE
end
const AxisCount = length(instances(AxisEnums))
zero_axis_vector() = MVector{AxisCount,Float32}(zeros(AxisCount))

@enum ActionEnums begin
    FORWARD = 1
    BACKWARD
    LEFT
    RIGHT
    TURNLEFT
    TURNRIGHT
end
const ActionCount = length(instances(ActionEnums))
zero_action_vector() = MVector{ActionCount,Int}(zeros(ActionCount))

"""
    get_button_map(input_map::Union{Dict, OrderedDict})::SMatrix

Create a nx2 SMatrix representing the button map based on the given input map.

Parameters:
- input_map: The button map represented as a dictionary.

Returns:
- SMatrix: The button map represented as an SMatrix.
"""
function get_button_map(input_map::Union{Dict,OrderedDict})::SMatrix
    n = length(input_map)
    map = Matrix{Int}(undef, n, 2)
    
    # Determine the current module to use for getting properties
    current_module = isdefined(Main, :AvesAscend) ? AvesAscend : Main

    for (i, (key, action)) in enumerate(input_map)
        # Determine the button value based on the key type
        button = if key isa String
            getfield(GLFW, Symbol(key)) |> Int
        elseif key isa Int
            key
        else
            error("Invalid key type: expected String or Int")
        end
        
        # Determine the action value
        action = getproperty(current_module, Symbol(action)) |> Int

        map[i, 1] = button
        map[i, 2] = action
    end

    return SMatrix{n,2}(map)
end

function get_button_map(input_map::Nothing)::SMatrix
    return SMatrix{0,2,Int}([])
end

function get_joystick_enums(joystick::Union{Dict,OrderedDict})::Dict
    joy_enums = Dict()
    for joy_index in 0:MAX_JOYSTICKS
        glfw_joy_num = GLFW.Joystick(joy_index)
        joy_name = GLFW.GetJoystickName(glfw_joy_num)

        if joy_name in keys(joystick)
            joy_enums[joy_name] = glfw_joy_num
        end
    end
    return joy_enums
end

function symbolize(map::Dict)::Dict
    Dict(index => Symbol(button) for (index, button) in map)
end

function symbolize(map::Nothing)::Dict
    return Dict()
end

struct JoystickMap
    enum::GLFW.Joystick
    buttons::SMatrix
    axes::SMatrix
end

function get_joystick_map(input_map::Union{Dict,OrderedDict})::Dict
    joy_enums = get_joystick_enums(input_map)

    joystick_map = Dict()
    for (joy_name, joy_info) in input_map
        joystick_map[joy_name] = JoystickMap(
            joy_enums[joy_name],
            get_button_map(joy_info["button"]),
            get_button_map(joy_info["axis"]),
        )
    end

    return joystick_map
end

mutable struct InputMap
    keyboard
    mouse
    joystick
end

function InputMap()
    input_map = YAML.load_file("./src/input/input_map.yml")

    keyboard = get_button_map(input_map["keyboard"])
    mouse = get_button_map(input_map["mouse"]["button"])
    joystick = get_joystick_map(input_map["joystick"])

    return InputMap(keyboard, mouse, joystick)
end

function get_actions(input_map::InputMap, window::GLFW.Window)::MVector
    # Process keyboard actions
    keyboard_actions = zero_action_vector()
    for (button, action) in eachrow(input_map.keyboard)
        key_status = GLFW.GetKey(window, button)
        keyboard_actions[action] |= key_status
    end

    # Process joystick actions
    joystick_actions = zero_action_vector()
    for joystick in values(input_map.joystick)
        button_values = GLFW.GetJoystickButtons(joystick.enum)
        joystick_actions[joystick.buttons[:, 2]] .|= button_values[joystick.buttons[:, 1]]
    end

    # Combine and return both action types
    return keyboard_actions .| joystick_actions
end

function process_axes!(axes::MVector, input_map::InputMap)
    # Iterate through joysticks and update the axes values
    for joystick in values(input_map.joystick)
        axes_values = GLFW.GetJoystickAxes(joystick.enum)
        axes[joystick.axes[:, 2]] .= axes_values[joystick.axes[:, 1]]
    end
end


@component mutable struct PlayerInputMap
    input_map::InputMap
end

@component mutable struct Actions
    buttons::MVector
    axes::MVector
end

struct PlayerActions <: System end

function Overseer.update(::PlayerActions, l::AbstractLedger)
    GLFW.PollEvents()
    for e in @entities_in(l, PlayerInputMap && Actions && Window)
        e.buttons = get_actions(e.input_map, e.window)
        process_axes!(e.axes, e.input_map)
        # print("\r$(e.buttons) \t $(e.axes)")
    end
end