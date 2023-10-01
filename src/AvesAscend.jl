module AvesAscend
using Overseer

include("input/input_manager.jl")
# include("game/game_state.jl")
# include("rendering/rendering_manager.jl")

function main()
    println("Initializing Game")

    rendering_manager = RenderingManager()
    render_sys = RenderSystem(rendering_manager)
    player_actions_sys = PlayerActions(rendering_manager)

    ledger = Ledger(Stage(:simulation, [player_actions_sys, render_sys]))

    p1_actions = Entity(
        ledger, 
        PlayerInputMap(InputMap()), 
        Actions(zero_action_vector(), zero_axis_vector()),
        PlayerActions(rendering_manager)
    )

    println("Finished Initializing Game ")

    while !should_exit(rendering_manager)
        Overseer.update(ledger)
    end
    
    cleanup(rendering_manager)
end

export main

end