module PGAs

using StaticArrays, CliffordAlgebras

export pga_2d, pga_3d, pga_klein

struct PGASub
    indices_tuple::NTuple
    indices_vector::Vector
    indices_mvector::MVector
    new::Function
end

function PGASub(algebra_symbol::Symbol, indices_tuple::NTuple)
    algebra = CliffordAlgebra(algebra_symbol) |> typeof
    indices_length = length(indices_tuple)

    indices_vector = collect(indices_tuple)
    indices_mvector = MVector{indices_length,Int}(indices_tuple)

    function new(ntuple::NTuple{N,T})::MultiVector where {N,T<:Real}
        @assert N == indices_length
        return MultiVector(algebra, indices_tuple, ntuple)
    end

    function new(args::T...)::MultiVector where {T<:Real}
        return MultiVector(algebra, indices_tuple, ntuple(i -> args[i], indices_length))
    end

    function new(vec::AbstractVector{T})::MultiVector where {T<:Real}
        return MultiVector(algebra, indices_tuple, NTuple{indices_length, T}(vec))
    end

    return PGASub(indices_tuple, indices_vector, indices_mvector, new)
end

@kwdef struct PGA
    motor::PGASub
    line::PGASub
    rotor::PGASub
    translator::PGASub
    point::PGASub
    direction::PGASub
    # plane::PGASub
    sudoscalar::PGASub
    multivector::PGASub
end

function PGA(algebra::Symbol, indices_dict)
    sub_algebras = Dict()
    for (sub_alg, indices) in indices_dict
        @assert sub_alg in (:motor, :line, :rotor, :translator, :point, :direction, :plane, :sudoscalar, :multivector)
        sub_algebras[sub_alg] = PGASub(algebra, indices)
    end

    return PGA(;sub_algebras...)

end

const pga_3d = PGA(
    :PGA3D,
    Dict(
        :motor => (1, 6, 7, 8, 9, 10, 11, 16),
        :line => (6, 7, 8, 9, 10, 11),
        :rotor => (1, 6, 7, 8),
        :translator => (1, 9, 10, 11),
        :point => (2, 3, 4, 5),
        :direction => (2, 3, 4),
        # :plane => (12, 13, 14, 15),
        :sudoscalar => (16,),
        :multivector => (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16),
    )
)

const pga_klein = PGA(
    :Klein,
    Dict(
        :motor => (5, 6, 7, 8, 9, 10, 11, 12),
        :line => (6, 7, 8, 10, 11, 12),
        :rotor => (5, 6, 7, 8),
        :translator => (5, 10, 11, 12),
        :point => (1, 2, 3, 4),
        :direction => (2, 3, 4),
        # :plane => (13, 14, 15, 16),
        :sudoscalar => (9,),
        :multivector => (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16),
    )
)

const pga_2d = PGA(
    :PGA2D,
    Dict(
        :motor => (1, 5, 6, 7),
        :line => (2, 3, 4),
        :rotor => (1, 5),
        :translator => (1, 6, 7),
        :point => (5, 6, 7),
        :direction => (6, 7),
        :sudoscalar => (8,),
        :multivector => (1, 2, 3, 4, 5, 6, 7, 8),
    )
)
end