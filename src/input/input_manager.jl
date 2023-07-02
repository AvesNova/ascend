using GLFW, YAML

# Constants
const MAX_JOYSTICKS = 15

# Structures
mutable struct InputMap
    keyboard
    mouse
    joystick
end

mutable struct Action
    forward::Bool
    backward::Bool
    left::Bool
    right::Bool
    turn_left::Bool
    turn_right::Bool
end

function Action()
    return Action(0, 0, 0, 0, 0, 0)
end

mutable struct Axis 
    throttle_axis::Float32
    yaw_axis::Float32
    roll_axis::Float32
    pitch_axis::Float32
end

function Axis()
    return Axis(0, 0, 0, 0)
end

mutable struct InputManager
    input_map::InputMap
    actions::Action
    axes::Axis
end

function InputManager()
    return InputManager(InputMap(), Action(), Axis())
end

# Mapping functions
function get_button_map(input_map::Dict)::Dict
    glfw_map = Dict()
    for (key, action) in input_map
        glfw_key = getfield(GLFW, Symbol(key))
        glfw_map[glfw_key] = Symbol(action)
    end
    return glfw_map
end

function get_button_map(input_map::Nothing)::Dict
    return Dict()
end

function get_joystick_enums(joystick::Dict)::Dict
    max_joysticks = 15
    joy_enums = Dict()
    for joy_index in 0:max_joysticks
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

function get_joystick_map(input_map::Dict)::Dict
    joy_enums = get_joystick_enums(input_map)

    joystick_map = Dict()
    for (joy_name, joy_info) in input_map
        joystick_map[joy_name] = Dict(
            :button => symbolize(joy_info["button"]),
            :axis => symbolize(joy_info["axis"]),
            :enum => joy_enums[joy_name],
        )
    end

    return joystick_map
end

function InputMap()
    input_map = YAML.load_file("./src/input/input_map.yml")

    keyboard = get_button_map(input_map["keyboard"])
    mouse = get_button_map(input_map["mouse"]["button"])
    joystick = get_joystick_map(input_map["joystick"])

    return InputMap(keyboard, mouse, joystick)
end

# Action handling functions
function merge_actions(keyboard_actions::Action, joystick_actions::Action)::Action
    merged_actions = Action()

    for field_name in fieldnames(Action)
        keyboard_value = getfield(keyboard_actions, field_name)
        joystick_value = getfield(joystick_actions, field_name)
        setfield!(merged_actions, field_name, joystick_value || keyboard_value)
    end
    return merged_actions
end

function get_actions(input_map::InputMap, window::GLFW.Window)::Action
    keyboard_actions = Action()
    for (button, action) in input_map.keyboard
        key_status = GLFW.GetKey(window, button)
        setfield!(keyboard_actions, action, key_status)
    end

    joystick_actions = Action()
    for joystick in values(input_map.joystick)
        button_values = GLFW.GetJoystickButtons(joystick[:enum])
        for (button_index, button_symbol) in joystick[:button]
            setfield!(joystick_actions, button_symbol, Bool(button_values[button_index]))
        end
    end

    return merge_actions(keyboard_actions, joystick_actions)
end

function process_axes!(axes::Axis, input_map::InputMap)
    for joystick in values(input_map.joystick)
        axes_values = GLFW.GetJoystickAxes(joystick[:enum])
        for (axis_index, axis_symbol) in joystick[:axis]
            setfield!(axes, axis_symbol, axes_values[axis_index])
        end
    end
end

# Main input processing function
function process_input(input_manager::InputManager, window::GLFW.Window)
    GLFW.PollEvents()
    input_manager.actions = get_actions(input_manager.input_map, window)
    process_axes!(input_manager.axes, input_manager.input_map)
    print("\r$(input_manager.actions) \t $(input_manager.axes)")
end
