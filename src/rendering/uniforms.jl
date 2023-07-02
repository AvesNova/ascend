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

function struct_to_buffer_data(s::T)::Vector{Float32} where T
    elements = fieldnames(T)
    buffer_data = Float32[]

    for element in elements
        val = getfield(s, element)

        if val isa Number
            push!(buffer_data, Float32(val))

        elseif val isa AbstractVector
            append!(buffer_data, Float32.(val))

        elseif val isa AbstractMatrix
            # Flatten the matrix in column major order
            append!(buffer_data, Float32.(vec(val)))
        end
    end

    return buffer_data
end

function struct_to_buffer_data(s::Vector{Float32})::Vector{Float32}
    return s
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

# Set a shader storage block with a new buffer data
function set_shader_storage_block(prog::GLA.Program, block_name::String, data::Any)
    buffer_data = struct_to_buffer_data(data)
    
    # Create a buffer for the data
    buffer_id = GLuint[0]
    glGenBuffers(1, buffer_id)

    # Bind this buffer and upload data
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffer_id[1])
    glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(buffer_data), buffer_data, GL_DYNAMIC_DRAW)

    # Get the index of the shader storage block in the shader
    block_index = glGetProgramResourceIndex(prog.id, GL_SHADER_STORAGE_BLOCK, block_name)

    # Check if the shader storage block was found
    if block_index != GL_INVALID_INDEX
        # Choose a binding point
        binding_point = 0

        # Bind the buffer to this binding point
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding_point, buffer_id[1])

        # Tell the shader to use this binding point for the shader storage block
        glShaderStorageBlockBinding(prog.id, block_index, binding_point)
    else
        @error "Shader storage block $block_name not found in the shader."
    end
end