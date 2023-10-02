using ModernGL, GLFW, GeometryTypes, Overseer
import GLAbstraction as GLA
include("shader_manager.jl")
include("uniforms.jl")

function create_window()
    # Create the window. This sets all the hints and makes the context current.
    window = GLFW.Window(name="Aves Ascention", resolution=(800,600))
    GLFW.MakeContextCurrent(window)
    GLA.set_context!(window)
    return window
end

function create_gla_program()
    vertex_shader = get_vertex_shader()
    fragment_shader = get_fragment_shader()
    gla_program = GLA.Program(vertex_shader, fragment_shader)
    return gla_program
end

function get_default_vertices()
    # The positions of the vertices in our rectangle
    vertex_positions = Point{2,Float32}[(-1.0,  1.0),     # top-left
                                        ( 1.0,  1.0),     # top-right
                                        ( 1.0, -1.0),     # bottom-right
                                        (-1.0, -1.0)]     # bottom-left

    elements = Face{3,UInt32}[(0,1,2),          # the first triangle
                              (2,3,0)]          # the second triangle

    return vertex_positions, elements
end

@pooled_component struct Window
    window::GLFW.Window
end

function Window()
    println("creating window")
    window = create_window()
    return Window(window)
end

@pooled_component struct RenderingManager
    gla_program::GLA.Program
    vertex_array_obj::GLA.VertexArray
end

function RenderingManager()
    println("creating rendering manager")
    gla_program = create_gla_program()
    vertex_positions, elements = get_default_vertices()
    buffers = GLA.generate_buffers(gla_program, GLA.GEOMETRY_DIVISOR; position = vertex_positions)
    vertex_array_obj = GLA.VertexArray(buffers, elements)
    return RenderingManager(gla_program, vertex_array_obj)
end

@pooled_component struct ObjectBuffer
    object_buffer
end

function ObjectBuffer()
    return ObjectBuffer(ones(Float32, 16))
end

struct RenderSystem <: System end

function Overseer.update(::RenderSystem, l::AbstractLedger)
    for e in @entities_in(l, Window && RenderingManager && ObjectBuffer)
        glClear(GL_COLOR_BUFFER_BIT)
        GLA.bind(e.gla_program)

        # put uniforms and buffers here
        # tex::Vector{Float32} = [1.0, 0.0, (cos(time()) + 1) / 2, 1.0]
        # u = GLA.uniform_location(e.gla_program, :tex)
        # if u != GLA.INVALID_UNIFORM
        #     glUniform4f(u, tex...)
        # end

        # test_buffer = ones(Float32, 16)
        # test_buffer[14:16] = e.pose[6:8]
        set_shader_storage_block(e.gla_program, "ObjectBuffer", e.object_buffer)

        GLA.bind(e.vertex_array_obj)
        GLA.draw(e.vertex_array_obj)
        GLA.unbind(e.vertex_array_obj)
        GLA.unbind(e.gla_program)
        GLFW.SwapBuffers(e.window)
    end
end

function should_exit(window::GLFW.Window)
    return GLFW.WindowShouldClose(window) || GLFW.GetKey(window, GLFW.KEY_ESCAPE) == GLFW.PRESS
end

function cleanup(window::GLFW.Window)
    GLFW.SetWindowShouldClose(window, true)
end
