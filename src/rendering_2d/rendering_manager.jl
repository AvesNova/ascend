using WGLMakie, Makie, Overseer


@pooled_component struct Window2D
    window::Figure
end

function Window2D()
    println("creating window")
    window = Figure()
    return Window2D(window)
end

struct RenderSystem2D <: System end

function Overseer.update(::RenderSystem2D, l::AbstractLedger)
    for e in @entities_in(l, Window2D)
        e.window
    end
end