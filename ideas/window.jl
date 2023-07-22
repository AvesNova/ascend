using ModernGL, GeometryTypes, GLAbstraction, GLFW, Random

const GLA = GLAbstraction

function create_window()
    # Create the window. This sets all the hints and makes the context current.
    window = GLFW.Window(name="Drawing polygons 5", resolution=(800,600))
    GLFW.MakeContextCurrent(window)
    GLA.set_context!(window)
    return window
end

function create_gla_program()
    vertex_source = read("shaders/vertex.glsl", String)
    fragment_source = read("shaders/fragment.glsl", String)

    vertex_shader = GLA.Shader(GL_VERTEX_SHADER, vertex_source)
    fragment_shader = GLA.Shader(GL_FRAGMENT_SHADER, fragment_source)

    gla_program = GLA.Program(vertex_shader, fragment_shader)
    return gla_program
end

function get_default_vertices()
    # The positions of the vertices in our rectangle
    vertex_positions = Point{2,Float32}[(-1.0,  1.0),     # top-left
                                        ( 1.0,  1.0),     # top-right
                                        ( 1.0, -1.0),     # bottom-right
                                        (-1.0, -1.0)]     # bottom-left

    # The colors assigned to each vertex
    vertex_colors = Vec3f0[(0, 0, 0),                     # top-left
                           (0, 0, 0),                     # top-right
                           (0, 0, 0),                     # bottom-right
                           (0, 0, 0)]                     # bottom-left

    # Specify how vertices are arranged into faces
    # Face{N,T} type specifies a face with N vertices, with index type
    # T (you should choose UInt32), and index-offset O. If you're
    # specifying faces in terms of julia's 1-based indexing, you should set
    # O=0. (If you instead number the vertices starting with 0, set
    # O=-1.)
    elements = Face{3,UInt32}[(0,1,2),          # the first triangle
                            (2,3,0)]          # the second triangle
    return vertex_positions, vertex_colors, elements
end

struct Camera
    c1::Vector{Float32}
    c2::Vector{Float32}
end

function window()
    window = create_window()
    prog = create_gla_program()
    vertex_positions, vertex_colors, elements = get_default_vertices()

    # This geometry now has to be uploaded into OpenGL Buffers and attached to the correct points in a VertexArray, to be used in the above program. 
    # We first generate the Buffers and their BufferAttachmentInfos, for each of the buffers, from the corresponding names and data 
    buffers = GLA.generate_buffers(prog, GLA.GEOMETRY_DIVISOR, 
        position = vertex_positions, 
        color = vertex_colors,
    )

    # The GEOMETRY_DIVISOR distinguishes that this is geometry data and not possible uniform data used in instanced vertexarrays
    # Now we create a VertexArray from these buffers and use the elements as the indices
    vao = GLA.VertexArray(buffers, elements)
    # Draw until we receive a close event
    glClearColor(0,0,0,0)
    while !GLFW.WindowShouldClose(window)
        glClear(GL_COLOR_BUFFER_BIT)
        GLA.bind(prog)

        tex::Vector{Float32} = [1.0, 0.0, 0.0, 0.0]
        u = GLA.uniform_location(prog, :tex)
        if u != GLA.INVALID_UNIFORM
            glUniform4f(u, tex...)
        end

        camera = Camera([0.0, 1.0, 0.0, 0.0], [0.8, 0.5, 0.2, 0.0])
        # convert the data to a single vector
        buffer_data = vcat(camera.c1, camera.c2)

        # Create a buffer for the camera data
        camera_buffer_id = glGenBuffers(1)

        # Bind this buffer and upload data
        glBindBuffer(GL_UNIFORM_BUFFER, camera_buffer_id)
        glBufferData(GL_UNIFORM_BUFFER, sizeof(buffer_data), buffer_data, GL_STATIC_DRAW)

        # Get the index of the Camera uniform block in the shader
        camera_block_index = glGetUniformBlockIndex(prog.id, "Camera")

        # Check if the uniform block was found
        if camera_block_index != GL_INVALID_INDEX
            # Choose a binding point
            binding_point = 0

            # Bind the buffer to this binding point
            glBindBufferBase(GL_UNIFORM_BUFFER, binding_point, camera_buffer_id)

            # Tell the shader to use this binding point for the Camera uniform block
            glUniformBlockBinding(prog.id, camera_block_index, binding_point)
        end

        # Draw your geometry as before...



        GLA.bind(vao)
        GLA.draw(vao)
        GLA.unbind(vao) #optional in this case
        GLA.unbind(prog) #optional in this case
        GLFW.SwapBuffers(window)
        GLFW.PollEvents()
        if GLFW.GetKey(window, GLFW.KEY_ESCAPE) == GLFW.PRESS
            GLFW.SetWindowShouldClose(window, true)
        end
    end
    GLFW.DestroyWindow(window)  # needed if you're running this from the REPL
end

window()