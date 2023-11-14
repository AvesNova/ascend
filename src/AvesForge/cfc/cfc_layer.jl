using Flux, LinearAlgebra

struct CfC_Layer
    input_size::Integer
    state_size::Integer
    ff1::Dense
    ff2::Dense
    time_a::Dense
    time_b::Dense
end

function CfC_Layer(input_size::Integer, state_size::Integer)
    cat_size = input_size + state_size

    ff1 = Dense(cat_size => state_size)
    ff2 = Dense(cat_size => state_size)

    time_a = Dense(cat_size => state_size)
    time_b = Dense(cat_size => state_size)

    return CfC_Layer(input_size, state_size, ff1, ff2, time_a, time_b)
end

function (model::CfC_Layer)(input::AbstractArray, hidden_state::AbstractArray, time_spans)
    x = cat(input, hidden_state; dims=1)

    ff1 = tanh.(model.ff1(x))
    ff2 = tanh.(model.ff2(x))
    t_a = model.time_a(x)
    t_b = model.time_b(x)

    t_interp = sigmoid_fast(t_a * time_spans + t_b)
    new_hidden = ff1 .* (1.0 .- t_interp) + t_interp .* ff2

    return new_hidden, new_hidden
end

function (model::CfC_Layer)(input::AbstractArray, hidden_state::Nothing, time_spans)
    hidden_state_0 = zeros(model.state_size, size(input)[end])
    return model(input, hidden_state_0, time_spans)
end

Flux.@functor CfC_Layer


xx = rand(3, 5)

cc = CfC_Layer(3, 11)

xx_1, hh_1 = cc(xx, nothing, 13)
xx_2, hh_2 = cc(xx, hh_1, 13)
xx_3, hh_3 = cc(xx, hh_2, 13)
xx_4, hh_4 = cc(xx, hh_3, 13)
xx_5, hh_5 = cc(xx, hh_4, 13)
xx_6, hh_6 = cc(xx, hh_5, 13)
