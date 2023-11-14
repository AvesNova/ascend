

struct Wiring
    units::Integer
    adjacency_matrix::AbstractMatrix{Integer}
    sensory_adjacency_matrix::AbstractMatrix{Integer}
    input_dim::Integer
    output_dim::Integer
end

function Wiring()
    
end