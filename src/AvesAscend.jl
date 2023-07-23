module AvesAscend

include("input/input_manager.jl")
include("game/game_state.jl")
include("rendering/rendering_manager.jl")

function main()
    input_manager = InputManager()
    rendering_manager = RenderingManager()
    game_state = GameState()

    while !should_exit(rendering_manager)
        process_input!(input_manager, rendering_manager.window)
        update_game_state!(game_state, input_manager)
        render(rendering_manager, game_state)
    end
    
    cleanup(rendering_manager)
end

export main

end