using GLWMakie, Makie, Overseer

function create_window()
    window = Figure()
    return window
end

@pooled_component struct Window