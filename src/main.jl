include("game/game_state.jl")
include("input/input_manager.jl")
include("rendering/rendering_manager.jl")

function main()
    input_manager = InputManager()
    rendering_manager = RenderingManager()
    game_state = GameState()

    # main game loop
    while !should_exit(rendering_manager)
        # process input
        process_input!(input_manager, rendering_manager.window)

        # update the game state
        update_game_state!(game_state)

        # render the game
        render(rendering_manager, game_state)
    end

    # clean up before exit
    cleanup(rendering_manager)
    # cleanup(input_manager)
end

main()  # Call the main function
