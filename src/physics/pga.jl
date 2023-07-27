using CliffordAlgebras
using StaticArrays

"""
    @define_clifford_algebra_helpers(algebra_symbol, algebra_name, type_symbol, indices...)

Macro to define helper functions and constants for Clifford algebras.

This macro aids in generating constants and utility functions for specific
Clifford algebra types, facilitating the process of working with various algebra elements.

# Arguments
- `algebra_symbol::Symbol`: Symbol to represent the algebra (e.g., `KleinMotor`).
- `algebra_name::Symbol`: String form of the algebra's name (e.g., `pga`).
- `type_symbol::Symbol`: Symbol for the specific type within the algebra (e.g., `Motor`, `Line`).
- `indices...::Int...`: The indices defining the specific multivector type within the Clifford algebra.

# Generated Entities
For an invocation like `@define_clifford_algebra_helpers KleinMotor pga Motor 1 2 3 4`:
- Constants for indices in different formats (tuple, vector, mvector).
- A function to create a `MultiVector` from an `NTuple`.
- A function to create a `MultiVector` from varargs.
- A function to create a `MultiVector` from an `AbstractVector`.
"""
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

@define_clifford_algebra_helpers KleinMotor pga Motor 1 2 3 4 5 6 7 8
@define_clifford_algebra_helpers KleinMotor pga Line 2 3 4 6 7 8
@define_clifford_algebra_helpers KleinMotor pga Rotor 1 2 3 4
@define_clifford_algebra_helpers KleinMotor pga Translator 5 6 7 8

# coefficients(pga_line(1, 2, 3, 4, 5, 6), PGA_LINE_INDICES_MVECTOR)