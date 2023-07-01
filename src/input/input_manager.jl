mutable struct InputManager
    start_time
end

function InputManager()
    return InputManager(time())
end

function process_input(input_manager::InputManager)
    GLFW.PollEvents()
end