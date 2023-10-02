using Overseer

include("entities.jl")

function create_ledger()::Ledger
    ledger = Ledger(Stage(:simulation, [
        RenderSystem(), 
        PlayerActions(),
        CameraMover(),
    ]))
    return ledger
end

function create_entities!(ledger::Ledger)
    renderer = Entity(
        ledger,
        Window(),
        RenderingManager(),
    )
    
    # player_1 = Entity(
    #     ledger, 
    #     PlayerInputMap(InputMap()), 
    #     Actions(zero_action_vector(), zero_axis_vector()),
    # )
    # ledger[Window][player_1] = renderer

    camera = Entity(
        ledger,
        PlayerInputMap(InputMap()), 
        Actions(zero_action_vector(), zero_axis_vector()),
        Camera(),
        Pose([1,0,0,0,0,0,0,0]),
    )
    ledger[Window][camera] = renderer

end