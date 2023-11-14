using Overseer

# include("entities.jl")

function create_ledger()::Ledger
    ledger = Ledger(Stage(:simulation, [
        RenderSystem(), 
        PlayerActions(),
        CameraMover(),
        Pose2Buffer(),
    ]))
    return ledger
end

function create_entities!(ledger::Ledger)
    renderer = Entity(
        ledger,
        Window(),
        RenderingManager(),
        ObjectBuffer(),
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
        Pose([-1, 0, 0, 0, 0, 0, 0, -3]),
    )
    ledger[Window][camera] = renderer
    ledger[RenderingManager][camera] = renderer
    ledger[ObjectBuffer][camera] = renderer

end