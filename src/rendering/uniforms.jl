import GLAbstraction as GLA
using ModernGL

# Set a uniform variable with 4 components
function set_uniform(prog::GLA.Program, name::Symbol, value::Vector{Float32})
    location = GLA.uniform_location(prog, name)
    if location != GLA.INVALID_UNIFORM
        glUniform4f(location, value...)
    else
        @error "Uniform $name not found in the shader."
    end
end

function struct_to_buffer_data(s::T) where T
    elements = fieldnames(T)
    buffer_data = Float32[]

    for element in elements
        val = getfield(s, element)

        if val isa Number
            push!(buffer_data, Float32(val))
        elseif val isa AbstractVector
            append!(buffer_data, Float32.(val))
            # If length less than 4, pad with zeros
            for _ in 1:(4 - length(val))
                push!(buffer_data, 0.0f0)
            end
        elseif val isa AbstractMatrix
            # Flatten the matrix in column major order
            append!(buffer_data, Float32.(vec(val)))
            # If fewer than 16 elements, pad with zeros
            for _ in 1:(16 - length(val))
                push!(buffer_data, 0.0f0)
            end
        end
    end

    return buffer_data
end

# Set a uniform block with a new buffer data
function set_uniform_block(prog::GLA.Program, block_name::String, data::Any)
    buffer_data = struct_to_buffer_data(data)
    
    # Create a buffer for the data
    buffer_id = GLuint[0]
    glGenBuffers(1, buffer_id)

    # Bind this buffer and upload data
    glBindBuffer(GL_UNIFORM_BUFFER, buffer_id[1])
    glBufferData(GL_UNIFORM_BUFFER, sizeof(buffer_data), buffer_data, GL_DYNAMIC_DRAW)

    # Get the index of the uniform block in the shader
    block_index = glGetUniformBlockIndex(prog.id, block_name)

    # Check if the uniform block was found
    if block_index != GL_INVALID_INDEX
        # Choose a binding point
        binding_point = 0

        # Bind the buffer to this binding point
        glBindBufferBase(GL_UNIFORM_BUFFER, binding_point, buffer_id[1])

        # Tell the shader to use this binding point for the uniform block
        glUniformBlockBinding(prog.id, block_index, binding_point)
    else
        @error "Uniform block $block_name not found in the shader."
    end
end
