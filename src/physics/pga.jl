using CliffordAlgebras
import StaticArrays: MVector

# function normalize(mv::MultiVector)::MultiVector
#     return mv / norm(mv)
# end

#TODO: I don't think I need these anymore
# function Base.iterate(mv::MultiVector)
#     return iterate(vector(mv))
# end

# function Base.iterate(mv::MultiVector, state::Int)
#     return iterate(vector(mv), state)
# end

macro define_pga_type(type, indices...)
    indices_length = length(indices)
    type_str = string(type)
    TYPE_UPPER = uppercase(type_str)
    Type_Camel = uppercase(type_str[1]) * lowercase(type_str[2:end])

    @eval begin
        const $(Symbol(TYPE_UPPER * "_INDICES_TUPLE")) = $indices
        const $(Symbol(TYPE_UPPER * "_INDICES_VECTOR")) = $[indices...]
        const $(Symbol(TYPE_UPPER * "_INDICES_MVECTOR")) = $MVector{$indices_length,Int}($indices)

        function $(Symbol(Type_Camel))(ntuple::NTuple{N,T})::MultiVector where {N,T<:Real}
            @assert N == $indices_length
            return MultiVector(PGA3D, $(Symbol(TYPE_UPPER * "_INDICES_TUPLE")), ntuple)
        end

        function $(Symbol(Type_Camel))(args::T...)::MultiVector where {T<:Real}
            MultiVector(PGA3D, $(Symbol(TYPE_UPPER * "_INDICES_TUPLE")), ntuple(i -> args[i], $indices_length))
        end

        function $(Symbol(Type_Camel))(vec::AbstractVector{T})::MultiVector where {T<:Real}
            return MultiVector(PGA3D, $(Symbol(TYPE_UPPER * "_INDICES_TUPLE")), NTuple{$indices_length, T}(vec))
        end
    end
end

@define_pga_type Multi 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
@define_pga_type Motor 1 6 7 8 9 10 11 16
@define_pga_type Line 6 7 8 9 10 11
@define_pga_type Rotor 1 6 7 8
@define_pga_type Translator 9 10 11 16
@define_pga_type Point 12 13 14 15
@define_pga_type Plane 2 3 4 5
