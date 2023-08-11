using GLFW, YAML
using StaticArrays


abstract type Controls end
abstract type ControlButtons <: Controls end
abstract type ControlAxes <: Controls end

const MAX_JOYSTICKS = 15

"""
    InputMap()

Data structure representing a map of user inputs from keyboard, mouse, and joystick.
"""
mutable struct InputMap
    keyboard
    mouse
    joystick
end

"""
    Action()

Data structure representing possible actions. 

Fields are Booleans representing whether a certain action is being performed.
"""
mutable struct Action
    forward::Bool
    backward::Bool
    left::Bool
    right::Bool
    turn_left::Bool
    turn_right::Bool
end

"""
    Action()

Create an Action object where all fields are initialized to false.
"""
function Action(;
    forward=false,
    backward=false,
    left=false,
    right=false,
    turn_left=false,
    turn_right=false,
)
    return Action(forward, backward, left, right, turn_left, turn_right)
end

"""
    Axis()

Data structure representing the throttle, yaw, roll, and pitch axes for input devices.

Fields are `Float32` representing the current state of each axis.
"""
mutable struct Axis 
    throttle_axis::Float32
    yaw_axis::Float32
    roll_axis::Float32
    pitch_axis::Float32
end

"""
    Axis()

Create an Axis object where all fields are initialized to 0.
"""
function Axis()
    return Axis(0, 0, 0, 0)
end

"""
    InputManager()

Data structure managing the current state of the user inputs.

Fields:
- `input_map::InputMap`: The current input map from keyboard, mouse, and joystick.
- `actions::Action`: The current state of possible actions.
- `axes::Axis`: The current state of throttle, yaw, roll, and pitch axes.
"""
mutable struct InputManager
    input_map::InputMap
    actions::Action
    axes::Axis
end

"""
    InputManager()

Create an InputManager object with newly initialized InputMap, Action, and Axis objects.
"""
function InputManager()
    return InputManager(InputMap(), Action(), Axis())
end

"""
    get_button_map(input_map::Dict)::Dict

Construct a GLFW button map from a given input map dictionary.

# Arguments
- `input_map::Dict`: A dictionary mapping input button names to actions.

# Returns
- A dictionary mapping GLFW keys to corresponding actions.
"""
function get_button_map(input_map::Dict)::Dict
    glfw_map = Dict()
    for (key, action) in input_map
        glfw_key = getfield(GLFW, Symbol(key))
        glfw_map[glfw_key] = Symbol(action)
    end
    return glfw_map
end

"""
    get_button_map(input_map::Nothing)::Dict

Return an empty dictionary when the input map is `Nothing`.

# Arguments
- `input_map::Nothing`: An absence of an input map.

# Returns
- An empty dictionary.
"""
function get_button_map(input_map::Nothing)::Dict
    return Dict()
end

"""
    get_joystick_enums(joystick::Dict)::Dict

Construct a dictionary mapping joystick names to their GLFW enum equivalents.

# Arguments
- `joystick::Dict`: A dictionary with joystick names as keys.

# Returns
- A dictionary mapping joystick names to their corresponding GLFW enum values.
"""
function get_joystick_enums(joystick::Dict)::Dict
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

"""
    symbolize(map::Dict)::Dict

Convert the values of a dictionary to Julia Symbols.

# Arguments
- `map::Dict`: A dictionary with indices mapped to buttons.

# Returns
- A dictionary with the same keys, but with the values converted to Symbols.
"""
function symbolize(map::Dict)::Dict
    Dict(index => Symbol(button) for (index, button) in map)
end

"""
    symbolize(map::Nothing)::Dict

Return an empty dictionary when the map is `Nothing`.

# Arguments
- `map::Nothing`: An absence of a map.

# Returns
- An empty dictionary.
"""
function symbolize(map::Nothing)::Dict
    return Dict()
end

"""
    get_joystick_map(input_map::Dict)::Dict

Create a mapping for joystick inputs based on the provided input map.

# Arguments
- `input_map::Dict`: The input map to be used for mapping joystick inputs to actions.

# Returns
- A `Dict` that contains the joystick mappings, with each joystick's name as the key and a `Dict` with
  the button mappings, axis mappings, and joystick enumeration as the value.
"""
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

"""
    InputMap()

Load the input mapping from the YAML file `./src/input/input_map.yml`.

# Returns
- An `InputMap` struct that contains the mappings for keyboard, mouse, and joystick inputs.
"""
function InputMap()
    input_map = YAML.load_file("./src/input/input_map.yml")

    keyboard = get_button_map(input_map["keyboard"])
    mouse = get_button_map(input_map["mouse"]["button"])
    joystick = get_joystick_map(input_map["joystick"])

    return InputMap(keyboard, mouse, joystick)
end

"""
    merge_actions(keyboard_actions::Action, joystick_actions::Action)::Action

Merge the keyboard and joystick actions into a single `Action` struct.

# Arguments
- `keyboard_actions::Action`: Actions captured from keyboard inputs.
- `joystick_actions::Action`: Actions captured from joystick inputs.

# Returns
- An `Action` struct that is the result of merging the keyboard and joystick actions.

# Examples
```jldoctest
julia> keyboard_actions = Action(1,1,0,0,0,0)
julia> joystick_actions = Action(0,1,1,0,0,0)
julia> merge_actions(keyboard_actions, joystick_actions)
Action(true, true, true, false, false, false)
```
"""
function merge_actions(keyboard_actions::Action, joystick_actions::Action)::Action
    merged_actions = Action()

    for field_name in fieldnames(Action)
        keyboard_value = getfield(keyboard_actions, field_name)
        joystick_value = getfield(joystick_actions, field_name)
        setfield!(merged_actions, field_name, joystick_value || keyboard_value)
    end
    return merged_actions
end

"""
    get_actions(input_map::InputMap, window::GLFW.Window)::Action

Retrieve the actions from the keyboard and joystick inputs based on the provided input map.

# Arguments
- `input_map::InputMap`: The input map to be used for mapping inputs to actions.
- `window::GLFW.Window`: The active GLFW window that captures the inputs.

# Returns
- An `Action` struct that captures the current actions from the keyboard and joystick inputs.
"""
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

"""
    process_axes!(axes::Axis, input_map::InputMap)

Update the values of the `axes` object based on the joystick inputs in the `input_map`.

# Arguments
- `axes::Axis`: The `Axis` object to be updated.
- `input_map::InputMap`: The input map to be used for mapping joystick axes to actions.

# Side Effects
- The `axes` object is mutated in-place with the updated joystick axes values.
"""
function process_axes!(axes::Axis, input_map::InputMap)
    for joystick in values(input_map.joystick)
        axes_values = GLFW.GetJoystickAxes(joystick[:enum])
        for (axis_index, axis_symbol) in joystick[:axis]
            setfield!(axes, axis_symbol, axes_values[axis_index])
        end
    end
end

"""
    process_input(input_manager::InputManager, window::GLFW.Window)

Main function for processing all inputs. Polls the events, retrieves actions from the input map, 
processes joystick axes, and prints the current state of actions and axes.

# Arguments
- `input_manager::InputManager`: The `InputManager` object that contains the current input map, actions, and axes.
- `window::GLFW.Window`: The GLFW window where the inputs are being captured.

# Side Effects
- The `actions` and `axes` fields of the `input_manager` are updated.
- The current state of actions and axes is printed to the console.
"""
function process_input!(input_manager::InputManager, window::GLFW.Window)
    GLFW.PollEvents()
    input_manager.actions = get_actions(input_manager.input_map, window)
    process_axes!(input_manager.axes, input_manager.input_map)
    print("\r$(input_manager.actions) \t $(input_manager.axes)")
end


im.keyboard
im.mouse
im.joystick["CH PRO THROTTLE USB"][:button]

@enum TestEnum begin
    a_enum = 1
    b_enum
    c_enum
    d_enum
    e_enum
    f_enum
    g_enum
    h_enum
end


# mutable struct Test1
#     a::Float64
#     b::Float64
#     c::Float64
#     d::Float64
#     e::Float64
#     f::Float64
#     g::Float64
#     h::Float64

#     vec_out::MVector{8, Float64}
# end

# function Test1()
#     return Test1(rand(8)..., rand(8))
# end




# function test0(count)
#     t = Test1()
#     v::MVector{8, Float64} = [1.0:8.0...,]
#     for _ in 1:count
#         setfield!(t, :a, 1.23)
#         setfield!(t, :b, 1.23)
#         setfield!(t, :c, 1.23)
#         setfield!(t, :d, 1.23)
#         setfield!(t, :e, 1.23)
#         setfield!(t, :f, 1.23)
#         setfield!(t, :g, 1.23)
#         setfield!(t, :h, 1.23)

#         v .+= 1
#     end
# end

# function test_mvec(count)
#     t::MVector{8, Float64} = [1.0:8.0...,]
#     v::MVector{8, Float64} = [1.0:8.0...,]
#     for _ in 1:count
#         t[1] = 1.23
#         t[2] = 1.23
#         t[3] = 1.23
#         t[4] = 1.23
#         t[5] = 1.23
#         t[6] = 1.23
#         t[7] = 1.23
#         t[8] = 1.23

#         v .+= 1
#     end
# end

# function test_mvec_enum(count)
#     t::MVector{8, Float64} = [1.0:8.0...,]
#     v::MVector{8, Float64} = [1.0:8.0...,]
#     for _ in 1:count
#         t[a_enum |> Int] = 1.23
#         t[b_enum |> Int] = 1.23
#         t[c_enum |> Int] = 1.23
#         t[d_enum |> Int] = 1.23
#         t[e_enum |> Int] = 1.23
#         t[f_enum |> Int] = 1.23
#         t[g_enum |> Int] = 1.23
#         t[h_enum |> Int] = 1.23

#         v .+= 1
#     end
# end

# function test_vec(count)
#     t::Vector = [1.0:8.0...,]
#     v::Vector = [1.0:8.0...,]
#     for _ in 1:count
#         t[1] = 1.23
#         t[2] = 1.23
#         t[3] = 1.23
#         t[4] = 1.23
#         t[5] = 1.23
#         t[6] = 1.23
#         t[7] = 1.23
#         t[8] = 1.23

#         v .+= 1
#     end
# end

# function test1(count)
#     t = Test1()
#     v::MVector{8, Float64} = [1.0:8.0...,]
#     for _ in 1:count
#         setfield!(t, :a, 1.23)
#         setfield!(t, :b, 1.23)
#         setfield!(t, :c, 1.23)
#         setfield!(t, :d, 1.23)
#         setfield!(t, :e, 1.23)
#         setfield!(t, :f, 1.23)
#         setfield!(t, :g, 1.23)
#         setfield!(t, :h, 1.23)

#         v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8] = t.a, t.b, t.c, t.d, t.e, t.f, t.g, t.h
#         v .+= 1
#     end
# end

# function vec!(v::MVector, t::Test1)::Nothing
#     v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8] = t.a, t.b, t.c, t.d, t.e, t.f, t.g, t.h
#     nothing
# end

# function test2(count)
#     t = Test1()
#     v::MVector{8, Float64} = [1.0:8.0...,]
#     for _ in 1:count
#         setfield!(t, :a, 1.23)
#         setfield!(t, :b, 1.23)
#         setfield!(t, :c, 1.23)
#         setfield!(t, :d, 1.23)
#         setfield!(t, :e, 1.23)
#         setfield!(t, :f, 1.23)
#         setfield!(t, :g, 1.23)
#         setfield!(t, :h, 1.23)

#         vec!(v, t)
#         v .+= 1
#     end
# end


# mycount = 1e9
# @time test0(mycount)
# @time test_mvec(mycount)
# @time test_mvec_enum(mycount)
# # @time test_vec(mycount)
# @time test1(mycount)
# @time test2(mycount)
