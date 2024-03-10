using StaticArrays, CliffordAlgebras

macro define_clifford_algebra_helpers(algebra_symbol, algebra_name, type_symbol, indices...)
    indices_length = length(indices)

    type_str = string(type_symbol)
    TYPE_UPPER = uppercase(type_str)
    type_lower = lowercase(type_str)

    algebra_str = string(algebra_name)
    ALGEBRA_UPPER = uppercase(algebra_str) * "_"
    algebra_lower = lowercase(algebra_str) * "_"

    algebra = CliffordAlgebra(algebra_symbol) |> typeof

    @eval begin
        const $(Symbol(ALGEBRA_UPPER * TYPE_UPPER * "_INDICES_TUPLE")) = $indices
        const $(Symbol(ALGEBRA_UPPER * TYPE_UPPER * "_INDICES_VECTOR")) = $[indices...]
        const $(Symbol(ALGEBRA_UPPER * TYPE_UPPER * "_INDICES_MVECTOR")) = $MVector{$indices_length,Int}($indices)

        function $(Symbol(algebra_lower * type_lower))(ntuple::NTuple{N,T})::MultiVector where {N,T<:Real}
            @assert N == $indices_length
            return MultiVector($algebra, $(Symbol(ALGEBRA_UPPER * TYPE_UPPER * "_INDICES_TUPLE")), ntuple)
        end

        function $(Symbol(algebra_lower * type_lower))(args::T...)::MultiVector where {T<:Real}
            MultiVector($algebra, $(Symbol(ALGEBRA_UPPER * TYPE_UPPER * "_INDICES_TUPLE")), ntuple(i -> args[i], $indices_length))
        end

        function $(Symbol(algebra_lower * type_lower))(vec::AbstractVector{T})::MultiVector where {T<:Real}
            return MultiVector($algebra, $(Symbol(ALGEBRA_UPPER * TYPE_UPPER * "_INDICES_TUPLE")), NTuple{$indices_length, T}(vec))
        end
    end
end
