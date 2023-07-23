using CliffordAlgebras
import CliffordAlgebras: MultiVector, coefficients
using StaticArrays

# import Base: .*, ./

const PGA = CliffordAlgebra(:PGA3D)

# function MultiVector(
#     CA::Type{<:CliffordAlgebra},
#     BI::NTuple{K,Integer},
#     c::NTuple{K,T},
# ) where {K,T<:Real}
#     @assert length(BI) > 0 
#     #@assert issorted(BI) && allunique(BI)
#     MultiVector{CA,T,convert(NTuple{K,Int}, BI),K}(c)
# end

MultiVector(
    ca::CliffordAlgebra, 
    BI::NTuple{K,Integer}, 
    c::NTuple{K,T}
) where {K,T<:Real} = MultiVector(typeof(ca), BI, c)

function MultiVectorPGA(s::Real, e1::Real, e2::Real, e3::Real, e0::Real, 
    e12::Real, e13::Real, e23::Real, e10::Real, e02::Real, e30::Real, 
    e132::Real, e120::Real, e103::Real, e230::Real, e1230::Real)::MultiVector
    return MultiVector(
        PGA,
        (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16),
        (s, e1, e2, e3, e0, e12, e13, e23, e10, e02, e30, e132, e120, e103, e230, e1230)
    )
end

function MultiVectorPGA(vec::Vector)::MultiVector
    return MultiVectorPGA(vec...)
end

function BiVectorPGA(e12, e13, e23, e10, e02, e30)::MultiVector
    MultiVector(PGA, (6, 7, 8, 9, 10, 11), (e12, e13, e23, e10, e02, e30))
end

const MOTOR_PGA_INDICES_TUPLE = (1, 6, 7, 8, 9, 10, 11, 16)
const MOTOR_PGA_INDICES_VECTOR = [1, 6, 7, 8, 9, 10, 11, 16]
const MOTOR_PGA_INDICES_MVECTOR::MVector{8, Int} = [1, 6, 7, 8, 9, 10, 11, 16]

function MotorPGA(motor::NTuple{N,T})::MultiVector where {N,T<:Real}
    @assert N == 8
    return MultiVector(PGA, MOTOR_PGA_INDICES_TUPLE, motor)
end

function MotorPGA(motor::T...)::MultiVector where T<:Real
    return MotorPGA(ntuple(i -> motor[i], 8))
end

function MotorPGA(motor::Union{Vector{T}, MVector{8,T}})::MultiVector where T<:Real
    return MotorPGA(NTuple{8, T}(motor))
end

function MotorPGA()::MultiVector
    return MultiVector(PGA, 1.0)
end

const LINE_PGA_INDICES_TUPLE = (6, 7, 8, 9, 10, 11)
const LINE_PGA_INDICES_VECTOR = [6, 7, 8, 9, 10, 11]
const LINE_PGA_INDICES_MVECTOR::MVector{6, Int} = [6, 7, 8, 9, 10, 11]

function LinePGA(line::NTuple{N,T})::MultiVector where {N,T<:Real}
    @assert N == 6
    return MultiVector(PGA, LINE_PGA_INDICES_TUPLE, line)
end

function LinePGA(line::T...)::MultiVector where T<:Real
    return LinePGA(ntuple(i -> line[i], length(line)))
end

function LinePGA(line::Union{Vector{T}, MVector{6,T}})::MultiVector where T<:Real
    return LinePGA(NTuple{6, T}(line))
end

function RotorPGA(rotor::NTuple{N,T})::MultiVector where {N,T<:Real}
    @assert N == 5
    return MultiVector(PGA, (1, 6, 7, 8, 9), rotor)
end

function RotorPGA(rotor::T...)::MultiVector where T<:Real
    return RotorPGA(ntuple(i -> rotor[i], length(rotor)))
end

function RotorPGA(vec::Vector)::MultiVector
    return RotorPGA(Tuple(vec))
end

function TranslatorPGA(translator::NTuple{N,T})::MultiVector where {N,T<:Real}
    @assert N == 4
    return MultiVector(PGA, (8, 9, 10, 11, 16), translator)
end

function TranslatorPGA(translator::T...)::MultiVector where T<:Real
    return TranslatorPGA(ntuple(i -> translator[i], length(translator)))
end

function PointPGA(point::NTuple{N,T})::MultiVector where {N,T<:Real}
    @assert N == 4
    return MultiVector(PGA, (12, 13, 14, 15), point)
end

function PointPGA(point::T...)::MultiVector where T<:Real
    return PointPGA(ntuple(i -> point[i], length(point)))
end

function PointPGA(x::T, y::T, z::T)::MultiVector where T<:Real
    return PointPGA((x, y, z, 1))
end

function PlanePGA(plane::NTuple{N,T})::MultiVector where {N,T<:Real}
    @assert N == 4
    return MultiVector(PGA, (2, 3, 4, 5), plane)
end

function PlanePGA(plane::T...)::MultiVector where T<:Real
    return PlanePGA(ntuple(i -> plane[i], length(plane)))
end

function Base.broadcasted(::typeof(*), mv1::MultiVector{CA,Ta,BI}, mv2::MultiVector{CA,Tb,BI})::MultiVector where {CA,Ta,Tb,BI}
    return MultiVector(CA, BI, coefficients(mv1) .* coefficients(mv2))
end

function Base.broadcasted(::typeof(*), mv1::MultiVector{CA,Ta,BIa}, mv2::MultiVector{CA,Tb,BIb})::MultiVector where {CA,Ta,Tb,BIa,BIb}
    v1, v2 = vector(mv1), vector(mv2)
    l1, l2 = length(v1), length(v2)
    @assert l1 == l2
    return MultiVector(CA, (1:l1...,), Tuple(v1 .* v2))
end

function Base.broadcasted(::typeof(/), mv1::MultiVector{CA,Ta,BI}, mv2::MultiVector{CA,Tb,BI})::MultiVector where {CA,Ta,Tb,BI}
    return MultiVector(CA, BI, coefficients(mv1) ./ coefficients(mv2))
end

function Base.broadcasted(::typeof(/), mv1::MultiVector{CA,Ta,BIa}, mv2::MultiVector{CA,Tb,BIb})::MultiVector where {CA,Ta,Tb,BIa,BIb}
    v1, v2 = vector(mv1), vector(mv2)
    l1, l2 = length(v1), length(v2)
    @assert l1 == l2
    return MultiVector(CA, (1:l1...,), Tuple(v1 ./ v2))
end

function normalize(mv::MultiVector)::MultiVector
    return mv / norm(mv)
end

function Base.iterate(mv::MultiVector)
    return iterate(vector(mv))
end

function Base.iterate(mv::MultiVector, state::Int)
    return iterate(vector(mv), state)
end

"""
    coefficient(::MultiVector, n::NTuple)

Returns the multivector coefficients for the given basis vectors. Returns 0 if index is out of bounds.
"""
function coefficients(
    mv::MultiVector{CA,T}, 
    idxs::Union{NTuple, Vector, MVector}
)::Union{NTuple, Vector, MVector} where {CA,T}

    # Precompute base indices for efficiency
    bases = baseindices(mv)
    coeffs = getfield(mv, :c)

    # Use map to avoid unnecessary allocations
    return map(idxs) do idx
        n = findfirst(isequal(idx), bases)
        isnothing(n) ? zero(T) : coeffs[n]
    end
end


# ll = LinePGA(randn(6))
# rr = RotorPGA(randn(5))
# mm = MultiVectorPGA(randn(16))

# coefficients(ll, (3, 4, 5, 6, 7, 8))
# coefficients(ll, [3, 4, 5, 6, 7, 8])

# mm .* mm
# ll .* rr

# MultiVector(PGA, (1:16...,), Tuple(randn(16)))