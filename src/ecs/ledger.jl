using Overseer

include("entities.jl")

function create_ledger()::Ledger
    ledger = Ledger(Stage(:simulation, [
        RenderSystem(), 
        PlayerActions(),
        CameraMover(),
        KineticMover(),
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

    camera = Entity(
        ledger,
        PlayerInputMap(InputMap()), 
        Actions(zero_action_vector(), zero_axis_vector()),
        Camera(),
        Pose([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -2.0]),
    )

    box = Entity(
        ledger,
        Pose([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0]),
        Twist([0.10, 0.00, 0.01, 0.0, 0.0, 0.0]),
        Kinetics(pga_k_line(1.0, 1.0, 1.0, 2.0, 1.0, 3.0)),
    )
    
    ledger[Window][camera] = renderer
    ledger[RenderingManager][camera] = renderer
    ledger[ObjectBuffer][camera] = renderer
    ledger[ObjectBuffer][box] = renderer

end