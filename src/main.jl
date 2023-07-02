include("rendering/rendering_manager.jl")
include("input/input_manager.jl")
include("ui/menu.jl")

function main()
    # initialize the input manager
    input_manager = InputManager()

    # initialize the rendering manager
    rendering_manager = RenderingManager()

    # load the main menu
    # main_menu = MainMenu()
    # load(main_menu)

    # main game loop
    while !should_exit(rendering_manager)
        # process input
        process_input(input_manager, rendering_manager.window)

        # update the game state
        # update_game_state()

        # render the game
        render(rendering_manager)
    end

    # clean up before exit
    cleanup(rendering_manager)
    # cleanup(input_manager)
    # unload(main_menu)
end

main()  # Call the main function
