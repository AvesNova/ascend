using Flux, LinearAlgebra
include("cfc.jl")

struct CfC_State{T<:Union{Tuple, NamedTuple, AbstractVector}}
    hidden_states::T
end

struct CfC_Chain{T<:Union{Tuple, NamedTuple, AbstractVector}}
    layer_sizes::AbstractVector
    layers::T
end

function CfC_Chain(layer_sizes::AbstractVector)
    [CfC_Layer(in_size, out_size) for (in_size, out_size) in zip(layer_sizes[1:end-1], layer_sizes[2:end])]
end

function split_array(array::AbstractArray, hidden_sizes::Union{Tuple, NamedTuple, AbstractVector}; dim::Integer = 1)
    end_idxs = cumsum(hidden_sizes)
    start_idxs = [1; end_idxs[1:end-1] .+ 1]

    return [selectdim(array, dim, i:j) for (i, j) in zip(start_idxs, end_idxs)]
end

function (model::CfC_Chain)(input, state, timespans)
    
end

function (model::CfC_Chain)(input, hx, timespans)
    hidden_states = split_array(hx, model.hidden_sizes)

    new_hidden_state = []
    for (i, hidden_state) in enumerate(hidden_states)
        input, _ = model.layers[i](input, hidden_state, timespans)
        push!(new_hidden_state, input)
    end
    new_hidden_state = cat(new_hidden_state; dims=2)

    return input, new_hidden_state
end

Flux.@functor CfC_Chain


ccc = CfC_Chain(2, [4, 4, 4])

rr = rand(4,4)

ccc(rr,rr, 1)