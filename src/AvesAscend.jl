module AvesAscend
using Overseer

include("AvesAscendGame/ledger.jl")

"""
    main()

Runs game
"""
function main()
    println("Initializing Game")

    ledger = create_ledger()
    create_entities!(ledger)

    println("Finished Initializing Game ")

    while !should_exit(ledger.components[Main.AvesAscend.Window][1].window)
        Overseer.update(ledger)
    end
    
    cleanup(ledger.components[Main.AvesAscend.Window][1].window)
end

export main

end